# ROS 2 Node Graph: Publisher/Subscriber Pattern

This diagram shows the communication flow between a publisher node and subscriber node in ROS 2.

```mermaid
graph LR
    subgraph Publisher Node
        P[Publisher<br/>minimal_publisher]
        PT[Timer<br/>1 Hz]
        PT --> P
    end

    subgraph ROS 2 Middleware DDS
        T[/chatter<br/>std_msgs/String/]
    end

    subgraph Subscriber Node
        S[Subscriber<br/>minimal_subscriber]
        SC[Callback<br/>listener_callback]
        S --> SC
    end

    P -->|publish| T
    T -->|deliver| S

    style P fill:#90EE90
    style S fill:#87CEEB
    style T fill:#FFD700
    style PT fill:#FFA07A
    style SC fill:#DDA0DD
```

## Explanation

- **Publisher Node**: Contains a timer that triggers publishing at 1 Hz
- **Topic** (`/chatter`): Message bus where String messages flow
- **Subscriber Node**: Listens to the topic and processes messages in a callback
- **DDS Middleware**: Handles message delivery between nodes (transparent to users)

## Usage in Docusaurus

To embed this diagram in a Docusaurus page, use:

\`\`\`markdown
```mermaid
graph LR
    subgraph Publisher Node
        P[Publisher]
    end
    ...
```
\`\`\`

Or reference this file:

\`\`\`markdown
import MermaidDiagram from '@site/static/img/module-1-ros2/diagrams/node-graph-pubsub.md';
\`\`\`
