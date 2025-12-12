import React, { useState, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const ContentDisplay = ({ contentId, title }) => {
  const { siteConfig } = useDocusaurusContext();
  const [content, setContent] = useState('');
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // In a real implementation, this would fetch content from an API
    // For now, we'll just simulate it with placeholder content
    const fetchContent = async () => {
      setLoading(true);
      
      // Simulate API delay
      await new Promise(resolve => setTimeout(resolve, 500));
      
      // Placeholder content - in a real implementation, this would come from an API
      const mockContent = `This is dynamically loaded content for ${title} (ID: ${contentId}).`;
      setContent(mockContent);
      setLoading(false);
    };

    fetchContent();
  }, [contentId, title]);

  return (
    <div className="content-display">
      <h3>{title}</h3>
      {loading ? (
        <div className="loading">Loading content...</div>
      ) : (
        <div className="content-body">
          <p>{content}</p>
          <div className="content-actions">
            <button 
              onClick={() => alert(`Sharing content: ${contentId}`)}
              className="button button--secondary button--sm"
            >
              Share
            </button>
            <button 
              onClick={() => alert(`Bookmarking content: ${contentId}`)}
              className="button button--secondary button--sm"
              style={{ marginLeft: '8px' }}
            >
              Bookmark
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ContentDisplay;