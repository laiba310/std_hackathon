import { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import AuthModal from '../../components/AuthModal';
import NavbarItem from '@theme/NavbarItem';

function AuthLink({ authType }) {
  const { isLoggedIn, logout } = useAuth();
  const [isModalOpen, setIsModalOpen] = useState(false);

  useEffect(() => {
    console.log('AuthLink: isModalOpen state changed to', isModalOpen);
  }, [isModalOpen]);

  const handleAuthClick = () => {
    console.log(`AuthLink: ${authType} clicked, setting isModalOpen to true`);
    setIsModalOpen(true);
  };

  const buttonText = authType === 'login' ? 'Login' : 'Sign Up';

  return (
    <>
      {isLoggedIn ? (
        <NavbarItem
          label="Logout"
          onClick={logout}
          position="right"
          className="navbar-auth-link"
        />
      ) : (
        <NavbarItem
          label={buttonText}
          onClick={handleAuthClick}
          position="right"
          className="navbar-auth-link"
        />
      )}
      <AuthModal isOpen={isModalOpen} onRequestClose={() => setIsModalOpen(false)} initialLoginState={authType === 'login'} />
    </>
  );
}

export default AuthLink;