import React, { useState, useEffect } from 'react';
import Modal from 'react-modal';
import { login, signup } from '../services/api';
import styles from './AuthModal.module.css';

function AuthModal({ isOpen, onRequestClose, initialLoginState = true }) {
  const [isLogin, setIsLogin] = useState(initialLoginState);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [experienceLevel, setExperienceLevel] = useState('beginner');
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // ✅ SSR-safe: run only in browser
  useEffect(() => {
    if (typeof window !== 'undefined') {
      Modal.setAppElement('#__docusaurus');
    }
  }, []);

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

      // ✅ SSR-safe reload
      if (typeof window !== 'undefined') {
        window.location.reload();
      }
    } catch (err) {
      setError(err?.message || 'Something went wrong. Please try again.');
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
            <path
              d="M13 1L1 13M1 1L13 13"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
            />
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
            {isLogin
              ? 'Sign in to continue learning'
              : 'Start your robotics journey today'}
          </p>
        </div>

        {/* Toggle */}
        <div className={styles.toggleContainer}>
          <button
            onClick={() => setIsLogin(true)}
            className={`${styles.toggleOption} ${
              isLogin ? styles.toggleActive : ''
            }`}
          >
            Login
          </button>
          <button
            onClick={() => setIsLogin(false)}
            className={`${styles.toggleOption} ${
              !isLogin ? styles.toggleActive : ''
            }`}
          >
            Sign Up
          </button>
          <div
            className={`${styles.toggleSlider} ${
              isLogin ? styles.sliderLeft : styles.sliderRight
            }`}
          />
        </div>

        {/* Error */}
        {error && (
          <div className={styles.errorMessage}>
            <span>{error}</span>
          </div>
        )}

        {/* Form */}
        <form onSubmit={handleSubmit} className={styles.form}>
          <input
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="your@email.com"
            required
            className={styles.input}
          />

          <input
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="••••••••"
            required
            minLength={6}
            className={styles.input}
          />

          {!isLogin && (
            <select
              value={experienceLevel}
              onChange={(e) => setExperienceLevel(e.target.value)}
              className={styles.select}
            >
              <option value="beginner">🎯 Beginner</option>
              <option value="intermediate">⚡ Intermediate</option>
              <option value="advanced">🚀 Advanced</option>
            </select>
          )}

          <button
            type="submit"
            disabled={isLoading}
            className={styles.submitButton}
          >
            {isLoading
              ? 'Please wait...'
              : isLogin
              ? 'Sign In'
              : 'Create Account'}
          </button>
        </form>

        {/* Footer */}
        <div className={styles.modalFooter}>
          <p>
            {isLogin ? "Don't have an account?" : 'Already have an account?'}
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
