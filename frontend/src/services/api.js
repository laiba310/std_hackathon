const API_URL = 'https://my-book-l7g8.onrender.com/api';

const handleResponse = async (response) => {
  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    // Use errorData.detail which is common in FastAPI
    throw new Error(errorData.detail || errorData.msg || response.statusText || 'Unknown API Error');
  }
  return response.json();
};

const authFetch = async (endpoint, method, body, contentType = 'application/json') => {
  const token = localStorage.getItem('jwt_token');
  const headers = {
    'Content-Type': contentType,
    'Authorization': token ? `Bearer ${token}` : undefined,
  };

  const config = {
    method: method,
    headers: headers,
    body: body ? JSON.stringify(body) : undefined,
  };

  // Remove Content-Type if form data is used (for Login)
  if (contentType === 'application/x-www-form-urlencoded') {
      delete config.headers['Content-Type'];
      config.body = body; // URLSearchParams object or string
  }

  return fetch(`${API_URL}${endpoint}`, config).then(handleResponse);
};

// --- LOGIN --- (Requires form-urlencoded)
export const login = async (email, password) => {
  const formBody = new URLSearchParams({
     username: email,
     password: password
  }).toString();

  const response = await fetch(`${API_URL}/auth/login`, {
     method: 'POST',
     headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
     body: formBody,
  });

  const data = await handleResponse(response);

  if (data.access_token) {
     localStorage.setItem('jwt_token', data.access_token);
  }

  return data;
};

// --- SIGNUP --- (Requires JSON)
export const signup = (data) => {
  return authFetch('/auth/signup', 'POST', data);
};

// --- CHAT ---
export const chat = (question) => {
  return authFetch('/chat', 'POST', { question });
};

// --- TRANSLATE ---
export const translate = (text, target_lang = 'Urdu') => {
  return authFetch('/translate', 'POST', { text, target_lang });
};
