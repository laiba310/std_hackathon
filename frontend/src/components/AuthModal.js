import React, { useState } from 'react';
import Modal from 'react-modal';
import { login, signup } from '../services/api';
import styles from './AuthModal.module.css';

Modal.setAppElement('#__docusaurus');

function AuthModal({ isOpen, onRequestClose, initialLoginState = true }) {
  const [isLogin, setIsLogin] = useState(initialLoginState);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [experienceLevel, setExperienceLevel] = useState('beginner');
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setIsLoading(true);
    
    try {
      if (isLogin) {
        await login(email, password);
        alert('Welcome back! Login successful.');
      } else {
        await signup({ email, password, experience_level: experienceLevel });
        alert('Account created! Please login.');
        setIsLogin(true);
      }
      onRequestClose();
      window.location.reload();
    } catch (err) {
      setError(err.message || 'Something went wrong. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Modal
      isOpen={isOpen}
      onRequestClose={onRequestClose}
      overlayClassName={styles.modalOverlay}
      className={styles.modalContent}
      closeTimeoutMS={300}
    >
      <div className={styles.modalContainer}>
        {/* Close Button */}
        <button
          onClick={onRequestClose}
          className={styles.closeButton}
          aria-label="Close modal"
        >
          <svg width="14" height="14" viewBox="0 0 14 14" fill="none">
            <path d="M13 1L1 13M1 1L13 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
          </svg>
        </button>

        {/* Header */}
        <div className={styles.modalHeader}>
          <div className={styles.headerIcon}>
            {isLogin ? '🔐' : '🚀'}
          </div>
          <h2 className={styles.modalTitle}>
            {isLogin ? 'Welcome Back' : 'Join the Journey'}
          </h2>
          <p className={styles.modalSubtitle}>
            {isLogin ? 'Sign in to continue learning' : 'Start your robotics journey today'}
          </p>
        </div>

        {/* Toggle Switch */}
        <div className={styles.toggleContainer}>
          <button
            onClick={() => setIsLogin(true)}
            className={`${styles.toggleOption} ${isLogin ? styles.toggleActive : ''}`}
          >
            Login
          </button>
          <button
            onClick={() => setIsLogin(false)}
            className={`${styles.toggleOption} ${!isLogin ? styles.toggleActive : ''}`}
          >
            Sign Up
          </button>
          <div className={`${styles.toggleSlider} ${isLogin ? styles.sliderLeft : styles.sliderRight}`} />
        </div>

        {/* Error Message */}
        {error && (
          <div className={styles.errorMessage}>
            <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
              <path d="M8 4V8M8 12H8.01M15 8C15 11.866 11.866 15 8 15C4.13401 15 1 11.866 1 8C1 4.13401 4.13401 1 8 1C11.866 1 15 4.13401 15 8Z" 
                stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
            </svg>
            <span>{error}</span>
          </div>
        )}

        {/* Form */}
        <form onSubmit={handleSubmit} className={styles.form}>
          <div className={styles.formGroup}>
            <div className={styles.inputContainer}>
              <svg className={styles.inputIcon} width="18" height="18" viewBox="0 0 24 24" fill="none">
                <path d="M3 8L10.8906 13.2604C11.5624 13.7083 12.4376 13.7083 13.1094 13.2604L21 8M5 19H19C20.1046 19 21 18.1046 21 17V7C21 5.89543 20.1046 5 19 5H5C3.89543 5 3 5.89543 3 7V17C3 18.1046 3.89543 19 5 19Z" 
                  stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
              </svg>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="your@email.com"
                required
                className={styles.input}
              />
            </div>
          </div>

          <div className={styles.formGroup}>
            <div className={styles.inputContainer}>
              <svg className={styles.inputIcon} width="18" height="18" viewBox="0 0 24 24" fill="none">
                <path d="M12 15V17M6 21H18C19.1046 21 20 20.1046 20 19V13C20 11.8954 19.1046 11 18 11H6C4.89543 11 4 11.8954 4 13V19C4 20.1046 4.89543 21 6 21Z" 
                  stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
                <path d="M17 11V7C17 4.23858 14.7614 2 12 2C9.23858 2 7 4.23858 7 7V11" 
                  stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
              </svg>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="••••••••"
                required
                className={styles.input}
                minLength="6"
              />
            </div>
          </div>

          {!isLogin && (
            <div className={styles.formGroup}>
              <div className={styles.selectContainer}>
                <svg className={styles.inputIcon} width="18" height="18" viewBox="0 0 24 24" fill="none">
                  <path d="M19 11H5M19 11C20.1046 11 21 11.8954 21 13V19C21 20.1046 20.1046 21 19 21H5C3.89543 21 3 20.1046 3 19V13C3 11.8954 3.89543 11 5 11M19 11V9C19 7.89543 18.1046 7 17 7M5 11V9C5 7.89543 5.89543 7 7 7M7 7V5C7 3.89543 7.89543 3 9 3H15C16.1046 3 17 3.89543 17 5V7M7 7H17" 
                    stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
                </svg>
                <select
                  value={experienceLevel}
                  onChange={(e) => setExperienceLevel(e.target.value)}
                  className={styles.select}
                >
                  <option value="beginner">🎯 Beginner - Just starting</option>
                  <option value="intermediate">⚡ Intermediate - Some experience</option>
                  <option value="advanced">🚀 Advanced - Expert level</option>
                </select>
              </div>
            </div>
          )}

          <button
            type="submit"
            className={`${styles.submitButton} ${isLoading ? styles.loading : ''}`}
            disabled={isLoading}
          >
            {isLoading ? (
              <div className={styles.spinner}></div>
            ) : (
              <>
                <span>{isLogin ? 'Sign In' : 'Create Account'}</span>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none">
                  <path d="M5 12H19M19 12L12 5M19 12L12 19" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </>
            )}
          </button>
        </form>

        {/* Footer */}
        <div className={styles.modalFooter}>
          <p className={styles.footerText}>
            {isLogin ? "Don't have an account?" : "Already have an account?"}
            <button
              onClick={() => setIsLogin(!isLogin)}
              className={styles.switchButton}
            >
              {isLogin ? 'Sign up here' : 'Login here'}
            </button>
          </p>
        </div>
      </div>
    </Modal>
  );
}

export default AuthModal;