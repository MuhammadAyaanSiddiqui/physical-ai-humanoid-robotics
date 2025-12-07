import React from 'react';
import './VideoEmbed.css';

/**
 * VideoEmbed component for YouTube videos
 * @param {Object} props
 * @param {string} props.videoId - YouTube video ID
 * @param {string} props.title - Video title for accessibility
 * @param {string} props.aspectRatio - Aspect ratio (default: '16:9')
 */
export default function VideoEmbed({ videoId, title, aspectRatio = '16:9' }) {
  const paddingTop = aspectRatio === '16:9' ? '56.25%' : '75%';

  return (
    <div className="video-embed-container" style={{ paddingTop }}>
      <iframe
        src={`https://www.youtube.com/embed/${videoId}`}
        title={title}
        frameBorder="0"
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
        allowFullScreen
        className="video-embed-iframe"
      ></iframe>
    </div>
  );
}
