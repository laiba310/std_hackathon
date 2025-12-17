# backend/ingest.py
import os
from uuid import uuid4
from qdrant_client import QdrantClient, models
from fastembed import TextEmbedding
from dotenv import load_dotenv
from pathlib import Path

load_dotenv()

# ENV CONFIG -----------------------------------------
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION", "textbook")
DOCS_DIR = os.getenv("TEXTBOOK_DOCS_DIR", "./frontend/docs")

# -----------------------------------------------------
def detect_embedding_dim(embedding_model):
    """Safely consume one sample embedding to detect vector dimension."""
    try:
        sample = embedding_model.embed(["hello world"])
        sample_list = list(sample)
        vec = sample_list[0]
        if hasattr(vec, "tolist"):
            vec = vec.tolist()
        return len(vec)
    except Exception as e:
        print("⚠️ Could not auto-detect embedding dimension:", e)
        return None

def ingest_docs():
    print("🚀 Starting Book Ingestion...")

    # 1) Connect to Qdrant --------------------------------
    try:
        client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        print("✅ Connected to Qdrant Cloud")
    except Exception as e:
        print(f"❌ Qdrant Connection Failed: {e}")
        return

    # 2) Load embedding model -----------------------------
    print("🧠 Loading embedding model (BAAI/bge-small-en-v1.5)...")
    embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")

    VECTOR_SIZE = detect_embedding_dim(embedding_model)
    if VECTOR_SIZE:
        print(f"ℹ️ Detected embedding dimension: {VECTOR_SIZE}")
    else:
        print("⚠️ Embedding dimension unknown — continuing without recreating collection.")

    # 3) Recreate Collection -------------------------------
    if VECTOR_SIZE:
        try:
            print(f"🗑️ Recreating collection '{COLLECTION_NAME}' with dim={VECTOR_SIZE} ...")
            client.recreate_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=VECTOR_SIZE,
                    distance=models.Distance.COSINE
                )
            )
        except Exception as e:
            print("⚠️ recreate_collection may have failed (API/version issue):", e)

    # 4) Locate Markdown Files -----------------------------
    current_dir = os.path.dirname(os.path.abspath(__file__))
    docs_path = os.path.join(current_dir, "..", "frontend", "docs")
    print(f"📂 Scanning folder: {Path(docs_path).resolve()}")

    documents = []
    metadatas = []
    ids = []
    counter = 0

    if not Path(docs_path).exists():
        print("❌ ERROR: docs folder not found:", docs_path)
        return

    for root, _, filenames in os.walk(docs_path):
        for filename in filenames:
            if filename.endswith(".md"):
                full_path = os.path.join(root, filename)
                try:
                    with open(full_path, "r", encoding="utf-8") as f:
                        content = f.read().strip()
                except Exception as e:
                    print("⚠️ Failed to read", full_path, e)
                    continue

                if len(content) < 50:
                    continue

                # simple chunking
                chunks = [content[i:i+1200] for i in range(0, len(content), 1200)]

                for chunk in chunks:
                    documents.append(chunk)
                    metadatas.append({
                        "document": chunk,
                        "source": filename,
                        "path": full_path
                    })
                    ids.append(str(uuid4()))
                    counter += 1

    if counter == 0:
        print("❌ ERROR: No markdown content found!")
        return

    print(f"📄 Found {counter} chunks. Generating embeddings and uploading in batches...")

    # 5) Create embeddings (consume generator)
    try:
        emb_gen = embedding_model.embed(documents)
        embeddings = list(emb_gen)
        # normalize vectors to plain lists
        vectors = []
        for v in embeddings:
            if hasattr(v, "tolist"):
                vectors.append(v.tolist())
            else:
                vectors.append(list(v))
    except Exception as e:
        print("❌ Embedding generation failed:", e)
        return

    # 6) Upload in small batches to avoid Qdrant timeout ----
    batch_size = 16

    for i in range(0, len(documents), batch_size):
        batch_vectors = vectors[i:i+batch_size]
        batch_metadata = metadatas[i:i+batch_size]
        batch_ids = ids[i:i+batch_size]

        print(f"▲ Uploading batch {i}..{i+len(batch_vectors)}")

        success = False
        for attempt in range(3):  # retry mechanism
            try:
                # try modern upsert first, fallback to upload_collection if needed
                if hasattr(client, "upsert"):
                    points = []
                    for idx, vec in enumerate(batch_vectors):
                        points.append({"id": batch_ids[idx], "vector": vec, "payload": batch_metadata[idx]})
                    client.upsert(collection_name=COLLECTION_NAME, points=points)
                elif hasattr(client, "upload_collection"):
                    client.upload_collection(
                        collection_name=COLLECTION_NAME,
                        vectors=batch_vectors,
                        payload=batch_metadata,
                        ids=batch_ids,
                    )
                else:
                    raise RuntimeError("Qdrant client has neither 'upsert' nor 'upload_collection' methods.")
                success = True
                break
            except Exception as e:
                print(f"⚠️ Upload attempt {attempt+1}/3 failed: {e}")

        if not success:
            print("❌ Upload failed after 3 retries — stopping.")
            return

        print(f"▲ Uploaded batch {i}..{i+len(batch_vectors)} (total uploaded so far: {i+len(batch_vectors)})")

    print(f"🎉 SUCCESS! Uploaded total {counter} chunks to Qdrant.")

# --------------------------------------------------------
if __name__ == "__main__":
    ingest_docs()
