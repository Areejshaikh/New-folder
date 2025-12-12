import React, { useState, useEffect, useRef } from 'react';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Function to get selected text
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request payload
      const requestData = {
        query: inputValue,
        session_id: 'session_' + Date.now(), // In a real app, use a proper session management
        selected_text: selectedText || null,
        context_page: window.location.pathname
      };

      // Call the backend API
      const response = await fetch('http://localhost:8000/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(requestData)
      });

      const data = await response.json();

      if (response.ok) {
        // Add bot response to the chat
        const botMessage = {
          id: Date.now() + 1,
          text: data.response,
          sender: 'bot',
          citations: data.citations,
          follow_up_questions: data.follow_up_questions,
          timestamp: new Date().toISOString()
        };

        setMessages(prev => [...prev, botMessage]);
      } else {
        // Handle error
        const errorMessage = {
          id: Date.now() + 1,
          text: 'Sorry, I encountered an error processing your request.',
          sender: 'bot',
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I\'m having trouble connecting to the server.',
        sender: 'bot',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const handleFollowUp = (question) => {
    setInputValue(question);
    setTimeout(() => {
      document.getElementById('chat-input').focus();
    }, 100);
  };

  return (
    <div className="chat-widget">
      {isOpen ? (
        <div className="chat-window">
          <div className="chat-header">
            <h3>AI Assistant</h3>
            <button className="close-btn" onClick={toggleChat}>
              ×
            </button>
          </div>
          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
              >
                <div className="message-content">
                  {message.text}
                </div>
                {message.citations && message.citations.length > 0 && (
                  <div className="citations">
                    <p><strong>Citations:</strong></p>
                    <ul>
                      {message.citations.map((citation, index) => (
                        <li key={index}>
                          <a href={citation.url} target="_blank" rel="noopener noreferrer">
                            {citation.title}
                          </a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
                {message.follow_up_questions && message.follow_up_questions.length > 0 && (
                  <div className="follow-up-questions">
                    <p><strong>You might also ask:</strong></p>
                    <ul>
                      {message.follow_up_questions.map((question, index) => (
                        <li key={index}>
                          <button onClick={() => handleFollowUp(question)}>
                            {question}
                          </button>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className="message bot-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <div className="chat-input-area">
            {selectedText && (
              <div className="selected-text-preview">
                <small>Selected: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"</small>
              </div>
            )}
            <div className="input-container">
              <textarea
                id="chat-input"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="Ask about the book content..."
                rows="1"
                className="chat-input"
              />
              <button
                onClick={sendMessage}
                disabled={isLoading || !inputValue.trim()}
                className="send-button"
              >
                Send
              </button>
            </div>
          </div>
        </div>
      ) : (
        <button className="chat-toggle" onClick={toggleChat}>
          <span>Ask AI</span>
        </button>
      )}
    </div>
  );
};

export default ChatWidget;