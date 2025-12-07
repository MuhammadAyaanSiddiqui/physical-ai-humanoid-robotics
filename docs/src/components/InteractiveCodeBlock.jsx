import React, { useState } from 'react';
import CodeBlock from '@theme/CodeBlock';
import './InteractiveCodeBlock.css';

/**
 * InteractiveCodeBlock component with copy-to-clipboard functionality
 * @param {Object} props
 * @param {string} props.language - Programming language for syntax highlighting
 * @param {string} props.children - Code content
 * @param {string} props.title - Optional title for the code block
 */
export default function InteractiveCodeBlock({ language, children, title }) {
  const [copied, setCopied] = useState(false);

  const handleCopy = () => {
    navigator.clipboard.writeText(children);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className="interactive-code-block">
      {title && <div className="code-block-title">{title}</div>}
      <CodeBlock language={language}>
        {children}
      </CodeBlock>
      <button
        className="copy-button"
        onClick={handleCopy}
        aria-label="Copy code to clipboard"
      >
        {copied ? '✓ Copied!' : 'Copy'}
      </button>
    </div>
  );
}
