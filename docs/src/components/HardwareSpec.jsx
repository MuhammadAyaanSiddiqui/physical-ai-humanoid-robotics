import React from 'react';
import './HardwareSpec.css';

/**
 * HardwareSpec component for displaying hardware requirement cards
 * @param {Object} props
 * @param {string} props.name - Hardware component name
 * @param {string} props.specs - Technical specifications
 * @param {string} props.price - Price range
 * @param {string} props.purpose - Why this hardware is needed
 * @param {boolean} props.optional - Whether this component is optional
 */
export default function HardwareSpec({ name, specs, price, purpose, optional = false }) {
  return (
    <div className={`hardware-spec-card ${optional ? 'optional' : 'required'}`}>
      <div className="hardware-header">
        <h4>{name}</h4>
        {optional && <span className="badge">Optional</span>}
      </div>
      <div className="hardware-body">
        <div className="spec-section">
          <strong>Specs:</strong>
          <p>{specs}</p>
        </div>
        <div className="spec-section">
          <strong>Price:</strong>
          <p>{price}</p>
        </div>
        <div className="spec-section">
          <strong>Purpose:</strong>
          <p>{purpose}</p>
        </div>
      </div>
    </div>
  );
}
