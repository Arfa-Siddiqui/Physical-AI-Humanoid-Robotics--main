import { useState, useEffect } from 'react';

interface TextSelection {
  text: string;
  hasSelection: boolean;
}

export const useTextSelection = (): TextSelection => {
  const [selection, setSelection] = useState<TextSelection>({
    text: '',
    hasSelection: false,
  });

  useEffect(() => {
    const handleSelectionChange = () => {
      const selectedText = window.getSelection()?.toString().trim() || '';

      setSelection({
        text: selectedText,
        hasSelection: selectedText.length > 0,
      });
    };

    document.addEventListener('selectionchange', handleSelectionChange);
    document.addEventListener('mouseup', handleSelectionChange);

    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleSelectionChange);
    };
  }, []);

  return selection;
};
