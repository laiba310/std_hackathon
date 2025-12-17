import { useRef, useEffect, useState } from 'react';
import ChapterToolbar from './ChapterToolbar';
import TranslationWrapper from './TranslationWrapper';

function ChapterContentWrapper({ children }) {
  const contentRef = useRef(null);
  const [currentLanguage, setCurrentLanguage] = useState('en');

  const handleToggleLanguage = (lang) => {
    setCurrentLanguage(lang);
  };

  useEffect(() => {
  }, [children]);

  return (
    <TranslationWrapper>
      {({ toggleTranslateLanguage, currentLanguage }) => (
        <div ref={contentRef}>
          <ChapterToolbar toggleTranslateLanguage={toggleTranslateLanguage} currentLanguage={currentLanguage} />
          {children}
        </div>
      )}
    </TranslationWrapper>
  );
}

export default ChapterContentWrapper;