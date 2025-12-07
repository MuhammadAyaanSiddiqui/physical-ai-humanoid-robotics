import React from 'react';
import Mermaid from '@theme/Mermaid';

/**
 * ArchitectureDiagram component for rendering Mermaid diagrams
 * @param {Object} props
 * @param {string} props.chart - Mermaid chart definition
 * @param {string} props.title - Optional title for the diagram
 */
export default function ArchitectureDiagram({ chart, title }) {
  return (
    <div className="architecture-diagram">
      {title && <h3>{title}</h3>}
      <Mermaid value={chart} />
    </div>
  );
}
