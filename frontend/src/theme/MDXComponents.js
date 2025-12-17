import React from 'react';
// Import the original mapper
import MDXComponents from '@theme-original/MDXComponents';
import ChapterContentWrapper from '@site/src/components/ChapterContentWrapper';
import ChapterToolbar from '@site/src/components/ChapterToolbar'; // Import ChapterToolbar

export default {
  // Re-use the default mapping
  ...MDXComponents,
  // Override the default wrapper component to include ChapterContentWrapper
  wrapper: ({ children }) => (
    <ChapterContentWrapper>
      {/* Render the original MDX content as children of the wrapper */}
      {children}
    </ChapterContentWrapper>
  ),
  // Make ChapterToolbar available as an MDX component
  ChapterToolbar,
};
