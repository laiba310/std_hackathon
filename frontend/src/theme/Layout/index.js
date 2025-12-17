// modified-
import Layout from '@theme-original/Layout';
import FloatingChatbot from '@site/src/components/FloatingChatbot';
import { AuthProvider } from '@site/src/contexts/AuthContext';

export default function LayoutWrapper(props) {
  return (
    <AuthProvider>
      <Layout {...props} />
      <FloatingChatbot />
    </AuthProvider>
  );
}