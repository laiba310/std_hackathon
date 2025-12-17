import React, { createContext, useContext, useState, useEffect } from 'react';

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [isLoggedIn, setIsLoggedIn] = useState(!!localStorage.getItem('jwt_token'));
  const [logoutRequested, setLogoutRequested] = useState(false);

  useEffect(() => {
    const token = localStorage.getItem('jwt_token');
    setIsLoggedIn(!!token);
  }, [logoutRequested]); // Update when logout is requested

  const login = () => {
    setIsLoggedIn(true);
  };

  const logout = () => {
    localStorage.removeItem('jwt_token');
    setIsLoggedIn(false);
    setLogoutRequested(prev => !prev); // Toggle to notify all components
  };

  return (
    <AuthContext.Provider value={{ isLoggedIn, login, logout }}>
      {children}
    </AuthContext.Provider>
  );
};