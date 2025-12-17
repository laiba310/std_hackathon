import React, { useEffect, useState, useCallback } from 'react';
import styles from './TranslationWrapper.module.css'; // For hiding Google Translate UI

const TranslationWrapper = ({ children }) => {
  const [isUrdu, setIsUrdu] = useState(false);
  const [currentLanguage, setCurrentLanguage] = useState('en'); // 'en' or 'ur'
  const [googleTranslateReady, setGoogleTranslateReady] = useState(false); // New state to track if GT is initialized

  // 1. Define the global callback for Google Translate initialization
  const googleTranslateElementInit = useCallback(() => {
    console.log("Google Translate: global googleTranslateElementInit callback triggered.");
    if (window.google && window.google.translate) {
      new window.google.translate.TranslateElement(
        {
          pageLanguage: 'en',
          layout: window.google.translate.TranslateElement.InlineLayout.SIMPLE,
          autoDisplay: false,
        },
        'google_translate_element'
      );
      setGoogleTranslateReady(true); // Mark Google Translate as ready
      console.log("Google Translate: Element initialized targeting JSX div and ready state set.");
    } else {
      console.log("Google Translate: window.google.translate not available during global init callback.");
    }
  }, []);

  // 2. useEffect to set global callback and load the script (runs once on mount)
  useEffect(() => {
    console.log("Google Translate: useEffect for global callback and script loading triggered.");

    // Ensure the global callback is defined before the script potentially calls it
    window.googleTranslateElementInit = googleTranslateElementInit;
    console.log("Google Translate: window.googleTranslateElementInit set globally.");

    // Check if the script is already in the document head to prevent duplicates
    const scriptId = 'google-translate-script';
    let existingScript = document.head.querySelector(`#${scriptId}`);

    if (!existingScript) {
      console.log("Google Translate: Injecting script...");
      const script = document.createElement('script');
      script.id = scriptId;
      script.src = '//translate.google.com/translate_a/element.js?cb=googleTranslateElementInit';
      script.async = true;
      script.onload = () => {
        console.log("Google Translate: Script loaded successfully.");
      };
      script.onerror = () => console.error("Google Translate: Script failed to load.");
      document.head.appendChild(script); // Append to head
    } else {
      console.log("Google Translate: Script already present in head.");
      // If script is already there, and we just mounted, try to ensure init is called if not already
      if (window.google && window.google.translate && !googleTranslateReady) {
        console.log("Google Translate: Script present, re-calling init if not ready.");
        googleTranslateElementInit();
      }
    }
  }, [googleTranslateElementInit, googleTranslateReady]); // googleTranslateReady is a dep here to catch cases where it's already true

  // 3. Polling logic to wait for the .goog-te-combo dropdown
  useEffect(() => {
    let interval;
    let attempts = 0;
    const maxAttempts = 100; // Try for up to 100 * 100ms = 10 seconds

    // Only start polling if Google Translate has successfully initialized its main element
    if (googleTranslateReady) {
      console.log("Google Translate: googleTranslateReady is true, starting polling for .goog-te-combo.");
      const pollForTranslateElement = () => {
        const selectElement = document.querySelector('.goog-te-combo');
        if (selectElement) {
          console.log("Google Translate: Select element (.goog-te-combo) found after polling.");
          const targetLang = isUrdu ? 'ur' : 'en';
          selectElement.value = targetLang;
          selectElement.dispatchEvent(new Event('change'));
          setCurrentLanguage(targetLang); // Update currentLanguage state
          clearInterval(interval);
        } else {
          attempts++;
          console.log(`Polling for .goog-te-combo. Attempt: ${attempts}`);
          if (attempts >= maxAttempts) {
            console.log("Google Translate: Max attempts reached. Select element not found.");
            clearInterval(interval);
          }
        }
      };

      interval = setInterval(pollForTranslateElement, 100);
    } else {
      console.log("Google Translate: Not ready yet, delaying polling start.");
    }

    return () => clearInterval(interval); // Cleanup on unmount or isUrdu change
  }, [isUrdu, googleTranslateReady]); // Depend on googleTranslateReady to restart polling when it becomes true

  const toggleTranslateLanguage = () => {
    setIsUrdu(prev => !prev);
  };

  return (
    <div className={styles.translationWrapper}>
      {/* This div is now part of the JSX and hidden using inline styles */}
      <div id="google_translate_element" style={{ position: 'absolute', opacity: 0, width: '1px', height: '1px', overflow: 'hidden' }}></div>
      {children({ toggleTranslateLanguage, currentLanguage })}
    </div>
  );
};

export default TranslationWrapper;