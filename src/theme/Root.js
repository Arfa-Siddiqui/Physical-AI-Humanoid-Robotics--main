import React from 'react';
import BookChat from '../components/BookChat';

export default function Root({children}) {
  // Use environment variable for API URL
  // For Vercel: Set REACT_APP_API_URL in Vercel dashboard
  // For local: Use http://localhost:8000
  const apiBaseUrl = typeof window !== 'undefined'
    ? (process.env.REACT_APP_API_URL || 'http://localhost:8000')
    : 'http://localhost:8000';

  return (
    <>
      {children}
      <BookChat apiBaseUrl={apiBaseUrl} />
    </>
  );
}
