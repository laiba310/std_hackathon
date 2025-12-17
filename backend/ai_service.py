import os
import inspect
import pathlib
import google.generativeai as genai
from qdrant_client import QdrantClient
from fastembed import TextEmbedding
from dotenv import load_dotenv
from typing import List, Any

load_dotenv()

# -------------------------
# Configuration
# -------------------------
GEMINI_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_KEY:
    raise RuntimeError("GEMINI_API_KEY missing in environment")
genai.configure(api_key=GEMINI_KEY)

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION", "textbook")
MODEL_NAME = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")
DOCS_DIR = os.getenv("TEXTBOOK_DOCS_DIR", "./frontend/docs")

# Initialize clients / models
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")


def get_model():
    """Return configured Gemini model instance."""
    return genai.GenerativeModel(MODEL_NAME)


# -------------------------
# Gemini response extractor
# -------------------------
def _extract_gemini_text(resp) -> str:
    """
    Robustly extract text from a google.generativeai response object.
    Tries common attributes: resp.text, resp.content, resp.candidates, resp.result, resp.outputs.
    Falls back to str(resp).
    """
    try:
        if hasattr(resp, "text") and resp.text:
            return getattr(resp, "text")
        if hasattr(resp, "content") and resp.content:
            return getattr(resp, "content")

        cand = getattr(resp, "candidates", None)
        if cand:
            try:
                first = cand[0]
                if hasattr(first, "output"):
                    return getattr(first, "output")
                if hasattr(first, "content"):
                    return getattr(first, "content")
                if isinstance(first, dict):
                    return first.get("content") or first.get("output") or str(first)
                return str(first)
            except Exception:
                pass

        res = getattr(resp, "result", None) or getattr(resp, "outputs", None)
        if res:
            try:
                r0 = res[0] if isinstance(res, (list, tuple)) else res
                if isinstance(r0, dict):
                    return r0.get("content") or r0.get("text") or str(r0)
                if hasattr(r0, "content"):
                    return getattr(r0, "content")
                if hasattr(r0, "text"):
                    return getattr(r0, "text")
                return str(r0)
            except Exception:
                pass
    except Exception:
        pass
    return str(resp)


# -------------------------
# Embedding conversion helper
# -------------------------
def _vector_to_list(v: Any) -> List[float]:
    """
    Convert embedding output into a plain Python list of floats.
    Handles numpy arrays, lists, or other iterables.
    """
    try:
        return v.tolist()  # numpy arrays or similar
    except Exception:
        try:
            return list(v)
        except Exception:
            return []


# -------------------------
# Local file loader (fallback when Qdrant payload has 'source' only)
# -------------------------
def _load_text_from_source(source_filename: str) -> str:
    """
    Try to read a local file by name from DOCS_DIR (and subfolders).
    """
    if not source_filename:
        return ""
    base = pathlib.Path(DOCS_DIR)
    # direct candidates
    candidates = [base / source_filename, base / (source_filename + ".md"), base / (source_filename + ".txt")]
    for c in candidates:
        if c.exists():
            try:
                return c.read_text(encoding="utf-8")
            except Exception:
                continue
    # recursive search fallback
    try:
        for p in base.rglob(source_filename):
            try:
                return p.read_text(encoding="utf-8")
            except Exception:
                continue
    except Exception:
        pass
    # fuzzy basename fallback
    name = pathlib.Path(source_filename).stem
    try:
        for p in base.rglob(f"{name}*"):
            try:
                if p.is_file():
                    return p.read_text(encoding="utf-8")
            except Exception:
                continue
    except Exception:
        pass
    return ""


# -------------------------
# Qdrant compatibility wrapper
# -------------------------
def _available_search_method(client: QdrantClient):
    """
    Return (method_name, callable) for the first matching search-like method on the Qdrant client.
    """
    candidates = [
        "search", "search_points", "search_point", "search_collection", "scroll", "query"
    ]
    for name in candidates:
        if hasattr(client, name):
            return name, getattr(client, name)
    # fallback: find first callable with search/query in name
    for attr in dir(client):
        if "search" in attr.lower() or "query" in attr.lower() or "scroll" in attr.lower():
            try:
                obj = getattr(client, attr)
                if callable(obj):
                    return attr, obj
            except Exception:
                continue
    return None, None


def _call_with_best_args(fn, collection: str, qvec: List[float], top: int, with_payload: bool):
    """
    Inspect fn signature and try likely keyword argument names.
    If keywords fail, fall back to positional call.
    """
    try:
        sig = inspect.signature(fn)
        param_names = list(sig.parameters.keys())
    except Exception:
        param_names = []

    # candidate kw names mapped to values we want to pass
    kw = {}

    # collection name
    for n in ("collection_name", "collection"):
        if n in param_names:
            kw[n] = collection
            break

    # vector param
    for n in ("query_vector", "query_vectors", "vector", "vectors", "vectors_"):
        if n in param_names:
            kw[n] = qvec
            break

    # top/limit param
    for n in ("limit", "top", "k", "n"):
        if n in param_names:
            kw[n] = top
            break

    # payload param
    for n in ("with_payload", "with_payloads", "with_vectors"):
        if n in param_names:
            kw[n] = with_payload
            break

    # Try calling with discovered kwargs
    if kw:
        try:
            print(f"Trying method with kwargs: {kw}")
            return fn(**kw)
        except TypeError as e:
            print(f"Kwargs call failed: {e} ; will attempt trial signatures.")

    # Try some fixed common combinations if signature didn't help
    try_signatures = [
        {"collection_name": collection, "query_vector": qvec, "limit": top, "with_payload": with_payload},
        {"collection_name": collection, "query_vector": qvec, "limit": top},
        {"collection": collection, "vector": qvec, "top": top, "with_payload": with_payload},
        {"collection": collection, "vector": qvec, "top": top},
        {"collection": collection, "vector": qvec, "limit": top},
        {"collection": collection, "vectors": [qvec], "limit": top},
        {"collection_name": collection, "vector": qvec, "limit": top},
    ]

    for tkw in try_signatures:
        try:
            print(f"Trying method with trial kwargs: {tkw}")
            return fn(**tkw)
        except TypeError as e:
            print(f"Trial kwargs failed: {e}")
            continue
        except Exception:
            # some other error (like collection not found) should be surfaced
            raise

    # Last resort: try positional args (collection, qvec, top)
    try:
        print("Trying method with positional args (collection, qvec, top)")
        return fn(collection, qvec, top)
    except Exception as e:
        raise RuntimeError(f"All attempts to call Qdrant method failed. Last error: {e}")


def _qdrant_search_safe(collection: str, query_vector: List[float], top: int = 3, with_payload: bool = True):
    method_name, method = _available_search_method(qdrant_client)
    if method is None:
        raise RuntimeError("No compatible search method found on QdrantClient. Available attrs: "
                           + ", ".join([a for a in dir(qdrant_client) if "search" in a.lower() or "scroll" in a.lower()]))
    print(f"Using Qdrant client method: {method_name} (callable: {method})")
    return _call_with_best_args(method, collection, query_vector, top, with_payload)


# -------------------------
# Helpers: Normalize hits (handle tuple (records, count), list, objects, dicts)
# -------------------------
def _normalize_hits(raw_hits):
    """
    Normalize hits returned by Qdrant into list of dicts:
      {"id": ..., "score": ..., "payload": {...}}
    """
    results = []
    if raw_hits is None:
        return results

    # If raw_hits is a 2-tuple like (records_list, total), extract the records_list
    if isinstance(raw_hits, (list, tuple)) and len(raw_hits) == 2 and isinstance(raw_hits[0], (list, tuple)):
        iterable = list(raw_hits[0])
    else:
        try:
            iterable = list(raw_hits)
        except Exception:
            iterable = [raw_hits]

    for hit in iterable:
        # If the hit itself is a tuple/list of (records, count) (nested), unwrap it
        if isinstance(hit, (list, tuple)) and len(hit) == 2 and isinstance(hit[0], (list, tuple)):
            records = hit[0]
            for r in records:
                if hasattr(r, "payload") or hasattr(r, "score") or hasattr(r, "id"):
                    payload = getattr(r, "payload", None)
                    if payload is None and hasattr(r, "payloads"):
                        payload = getattr(r, "payloads")
                    results.append({
                        "id": getattr(r, "id", None),
                        "score": getattr(r, "score", None),
                        "payload": payload or {}
                    })
                elif isinstance(r, dict):
                    results.append({
                        "id": r.get("id"),
                        "score": r.get("score"),
                        "payload": r.get("payload") or {}
                    })
                else:
                    results.append({"id": None, "score": None, "payload": {}})
            continue

        # object-like hit (attributes)
        if hasattr(hit, "payload") or hasattr(hit, "score") or hasattr(hit, "id"):
            payload = getattr(hit, "payload", None)
            if payload is None and hasattr(hit, "payloads"):
                payload = getattr(hit, "payloads")
            results.append({
                "id": getattr(hit, "id", None),
                "score": getattr(hit, "score", None),
                "payload": payload or {}
            })
            continue

        # dict-like hit
        if isinstance(hit, dict):
            results.append({
                "id": hit.get("id"),
                "score": hit.get("score"),
                "payload": hit.get("payload") or {}
            })
            continue

        # fallback
        results.append({"id": None, "score": None, "payload": {}})

    return results


# -------------------------
# Public functions
# -------------------------
def translate_text(text: str, target_lang: str = "Urdu") -> str:
    try:
        model = get_model()
        prompt = f"Translate to {target_lang}. Technical terms in English. Text: {text}"
        response = model.generate_content(prompt)
        return _extract_gemini_text(response).strip()
    except genai.types.BlockedPromptException as e:
        return f"Translation Error: Content blocked by safety filters. ({str(e)})"
    except genai.types.APIError as e:
        return f"Translation Error: Gemini API issue. Check API key and quota. ({str(e)})"
    except Exception as e:
        return f"Translation Error: An unexpected error occurred. ({str(e)})"


def get_chat_response(question: str) -> str:
    try:
        print(f"🔍 Searching Qdrant for: {question}")

        # 1) Embedding: consume generator safely
        emb_out = embedding_model.embed([question])
        try:
            emb_list = list(emb_out)
        except TypeError:
            emb_list = [emb_out]

        if not emb_list or len(emb_list) == 0:
            raise RuntimeError("Embedding produced no output.")

        query_vector_raw = emb_list[0]
        query_vector = _vector_to_list(query_vector_raw)
        if not query_vector:
            raise RuntimeError("Embedding produced empty vector.")

        # 2) Query Qdrant safely (ask for more top hits)
        raw_hits = _qdrant_search_safe(COLLECTION_NAME, query_vector, top=5, with_payload=True)

        # 3) Normalize hits
        hits = _normalize_hits(raw_hits)

        print("Raw hits (repr):", repr(raw_hits)[:1000])
        print("Normalized hits:", hits[:6])

        # 4) Extract textual context from payloads (collect documents & sources)
        texts = []
        source_files = []
        for h in hits:
            payload = h.get("payload") or {}
            if isinstance(payload, dict):
                # priority: document, then other keys
                if "document" in payload and payload["document"]:
                    texts.append(payload["document"])
                    source_files.append(payload.get("source"))
                    continue
                # fallback to other keys
                for key in ("text", "content", "body"):
                    if key in payload and payload[key]:
                        texts.append(payload[key])
                        source_files.append(payload.get("source"))
                        break
            else:
                payload_str = str(payload).strip()
                if payload_str:
                    texts.append(payload_str)

        context = "\n\n".join(texts).strip()
        if not context:
            context = "No specific context found in the book."

        print(f"📖 Context Found (first 400 chars): {context[:400]}")

        # 5) Primary prompt: encourage synthesis from context and require source listing
        model = get_model()
        prompt = f"""
You are an expert AI Robotics Professor. Use ONLY the provided CONTEXT to answer the student's question.
- If the answer can be directly or reliably INFERRED from the context, produce a short, clear answer (2-6 sentences).
- If the context contains partial information, COMBINE the parts from the context to create a correct concise answer.
- If the context has NO relevant information, respond exactly: "I cannot find this in the textbook."
- At the end, append a short "Sources:" line listing the filenames used (comma-separated). If no sources used, write "Sources: none".

Special rule about greetings:
- You MAY answer simple greetings (e.g., "hi", "hello", "salam") with a brief friendly reply (one sentence). For all other interaction, follow the textbook-only rules above.

CONTEXT:
{context}

Student Question: {question}
Answer:
"""
        response = model.generate_content(prompt)
        text = _extract_gemini_text(response).strip()
        print("🔔 Gemini response (primary):", text[:400])

        # 6) Fallback: if model refuses to answer, ask for an inferred answer (labeled)
        low = text.lower().strip()
        if low.startswith("i cannot find") or "cannot find this in the textbook" in low:
            fallback_prompt = f"""
You were given the textbook CONTEXT but could not find a direct answer. Now, using both the CONTEXT (if any) and general, accurate knowledge,
produce a short, clear answer to the student's question. PREPEND the answer with the exact phrase:
"INFERRED ANSWER (not directly in textbook):"
Then give the answer (2-4 sentences). After the answer, include a short line:
"Textbook sources checked: <comma-separated filenames or 'none'>"
Context used:
{context}
Question: {question}
Answer:
"""
            fallback_resp = model.generate_content(fallback_prompt)
            fallback_text = _extract_gemini_text(fallback_resp).strip()
            print("🔔 Gemini response (fallback):", fallback_text[:400])
            return fallback_text

        return text

    except Exception as e:
        print(f"❌ Chat Error: {str(e)}")
        return f"Error: {str(e)}"
