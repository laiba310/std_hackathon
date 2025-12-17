import { useState, useEffect } from 'react';
import { chat } from '../services/api';
import AuthModal from './AuthModal';
import styles from './FloatingChatbot.module.css';
import clsx from 'clsx';
import { FaTimes, FaCommentDots, FaPaperPlane } from 'react-icons/fa';

function FloatingChatbot() {
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    console.log('FloatingChatbot: isOpen', isOpen);
  }, [isOpen]);
  const [messages, setMessages] = useState([
    { text: "🤖 Hi! I'm your AI Professor. Ask me anything about the book content!", sender: 'ai' }
  ]);
  const [input, setInput] = useState('');
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [showSignup, setShowSignup] = useState(false);
  const [loading, setLoading] = useState(false);
  const [isLoggedIn, setIsLoggedIn] = useState(false); 

  useEffect(() => {
    setIsLoggedIn(!!localStorage.getItem('jwt_token'));
  }, []);

  useEffect(() => {
    console.log('FloatingChatbot: isModalOpen', isModalOpen);
    console.log('FloatingChatbot: showSignup', showSignup);
  }, [isModalOpen, showSignup]);

  const handleSendMessage = async () => {
    if (input.trim() === '') return;

    const userMessage = { text: input, sender: 'user' };
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await chat(input);
      const aiMessage = { text: response.answer, sender: 'ai' };
      setMessages((prev) => [...prev, aiMessage]);
    } catch (error) {
      if (error.message.includes('401') || error.response?.status === 401) {
        setIsModalOpen(true);
        setMessages((prev) => [...prev, { text: "🔒 Access Denied. Please login to chat.", sender: 'ai' }]);
      } else {
        setMessages((prev) => [...prev, { text: "⚠️ Error connecting to Brain.", sender: 'ai' }]);
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.headerTitle}>
              <FaCommentDots size={20}/> <span>AI Professor</span>
            </div>
            <div className={styles.headerControls}>
                {!isLoggedIn && (<span className={styles.loginLink} onClick={() => { console.log('Login button clicked'); setIsModalOpen(true); setShowSignup(true); }}>Login</span>)}
                <FaTimes style={{ cursor: 'pointer' }} onClick={() => setIsOpen(false)} />
            </div>
          </div>

          <div className={styles.messages}>
            {messages.map((msg, index) => (
              <div key={index} className={clsx(styles.message, styles[msg.sender])}>
                {msg.text}
              </div>
            ))}
            {loading && <div className={clsx(styles.message, styles.ai)}>Thinking... 🧠</div>}
          </div>

          <div className={styles.inputArea}>
            <div className={styles.inputWrapper}>
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
                placeholder="Type your question..."
                disabled={loading}
              />
            </div>
            <button className={styles.sendButton} onClick={handleSendMessage} disabled={loading}>
              <FaPaperPlane />
            </button>
          </div>
        </div>
      )}

      <button className={styles.toggleButton} onClick={() => setIsOpen(!isOpen)}>
        {isOpen ? <FaTimes size={24} /> : <FaCommentDots size={24} />}
      </button>

      <AuthModal isOpen={isModalOpen} onRequestClose={() => setIsModalOpen(false)} initialLoginState={!showSignup} />
    </div>
  );
}

export default FloatingChatbot;