import React from 'react';
import BookChat from '../components/BookChat';

export default function Root({children}) {
  return (
    <>
      {children}
      <BookChat />
    </>
  );
}
