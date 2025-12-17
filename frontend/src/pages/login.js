import React, { useState } from 'react';
import Layout from '@theme/Layout';
import AuthModal from '../components/AuthModal';

function LoginPage() {
  const [isModalOpen, setIsModalOpen] = useState(false);

  const handleCloseModal = () => {
    setIsModalOpen(false);
    window.location.href = '/';
  };

  const handleOpenModal = () => {
    setIsModalOpen(true);
  };

  return (
    <Layout title="Login">
      <main style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '60vh',
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
            Open Login
          </button>
        )}
        <AuthModal isOpen={isModalOpen} onRequestClose={handleCloseModal} initialLoginState={true} />
      </main>
    </Layout>
  );
}

export default LoginPage;