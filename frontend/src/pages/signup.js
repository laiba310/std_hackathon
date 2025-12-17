import React, { useState } from 'react';
import Layout from '@theme/Layout';
import AuthModal from '../components/AuthModal';

function SignupPage() {
  const [isModalOpen, setIsModalOpen] = useState(false); // Start with modal closed by default

  const handleCloseModal = () => {
    setIsModalOpen(false);
    window.location.href = '/'; // Redirect to home after closing
  };

  const handleOpenModal = () => {
    setIsModalOpen(true);
  };

  return (
    <Layout title="Sign Up">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '60vh', // Ensure it takes enough height to center content
        padding: '20px'
      }}>
        {!isModalOpen && (
          <button
            onClick={handleOpenModal}
            style={{
              padding: '10px 20px',
              fontSize: '18px',
              cursor: 'pointer',
              borderRadius: '8px',
              border: '1px solid #ccc',
              backgroundColor: '#f0f0f0',
              color: '#333'
            }}
          >
            Open Sign Up
          </button>
        )}
        <AuthModal isOpen={isModalOpen} onRequestClose={handleCloseModal} initialLoginState={false} />
      </main>
    </Layout>
  );
}

export default SignupPage;