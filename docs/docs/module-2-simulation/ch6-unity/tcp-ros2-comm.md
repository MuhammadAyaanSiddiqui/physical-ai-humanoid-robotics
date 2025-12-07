---
---

# Unity to ROS 2 Communication with the TCP Connector

With your robot imported and your connection configured, it's time to make Unity "talk" to ROS 2. This communication is handled by the **ROS TCP Connector** package, which allows C# scripts in Unity to publish and subscribe to ROS 2 topics.

## The Communication Architecture

The `ROS TCP Connector` in Unity and the `ros_tcp_endpoint` on the ROS 2 machine work together as a client-server pair.

1.  **Unity (Client)**: Your C# scripts use the `ROSConnection` object to register topics they want to publish or subscribe to. When you enter Play mode, Unity establishes a single, persistent TCP connection to the ROS 2 endpoint.
2.  **ROS 2 (Server)**: The `ros_tcp_endpoint` node listens for this incoming TCP connection. Once connected, it acts as a proxy:
    -   It subscribes to ROS 2 topics on behalf of Unity. When it receives a message, it forwards it over the TCP socket to Unity.
    -   It creates ROS 2 publishers on behalf of Unity. When it receives data from Unity over the TCP socket, it publishes it as a ROS 2 message.

```mermaid
graph TD
    subgraph "Unity Scene"
        A[C# Script<br>(Publisher)]
        B[C# Script<br>(Subscriber)]
        C[ROSConnection Singleton]
        A --> C
        B --> C
    end

    subgraph "ROS 2 Graph"
        F[ROS 2 Node]
        G[ROS 2 Topic]
        H[ROS 2 Node]
        I[ROS 2 Topic]
        F --> G
        I --> H
    end
    
    C -- TCP/IP Socket --> D[ros_tcp_endpoint]

    subgraph "TCP Proxy"
        D
    end

    D -- "Publishes to ROS 2 Topic" --> G
    D -- "Subscribes to ROS 2 Topic" --> I

    style C fill:#ccf,stroke:#333,stroke-width:2px
    style D fill:#cfc,stroke:#333,stroke-width:2px
```

This architecture is efficient because it uses a single TCP connection to multiplex all topic data, avoiding the overhead of creating multiple sockets.

## Publishing from Unity to ROS 2

Let's create a C# script that publishes a simple "Hello World" message from Unity to a ROS 2 topic.

1.  **Create a C# Script**: In your `Assets` folder, create a script named `RosPublisher.cs`.
2.  **Attach it to an object**: Create an empty `GameObject` in your scene (right-click in Hierarchy > Create Empty) and name it "RosManager". Drag your `RosPublisher.cs` script onto it.
3.  **Edit the script**:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Import the message type

public class RosPublisher : MonoBehaviour
{
    // ROS Connection
    private ROSConnection ros;
    
    // Topic name
    private string topicName = "unity_chatter";

    // Message to publish
    private StringMsg message;

    // Publish frequency (in seconds)
    private float publishMessageFrequency = 1.0f;
    private float timeElapsed;

    void Start()
    {
        // Get the ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        
        // Register the publisher
        ros.RegisterPublisher<StringMsg>(topicName);

        // Initialize the message
        message = new StringMsg("Hello from Unity!");
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // Update the message data (e.g., with a timestamp)
            message.data = $"Hello from Unity! Time: {Time.time}";

            // Publish the message
            ros.Publish(topicName, message);
            
            Debug.Log($"Published: {message.data}");

            // Reset the timer
            timeElapsed = 0;
        }
    }
}
```

**What this script does:**
-   In `Start()`, it gets the `ROSConnection` singleton and calls `RegisterPublisher<StringMsg>(topicName)`. This tells the `ros_tcp_endpoint` to create a publisher for the `unity_chatter` topic.
-   In `Update()`, which is called every frame, it uses a simple timer to publish a message once per second.
-   `ros.Publish(topicName, message)` sends the message data over the TCP socket to the endpoint, which then publishes it to the ROS 2 graph.

**To test it:**
1.  Run the `ros_tcp_endpoint` on your ROS 2 machine.
2.  Press Play in Unity.
3.  In your ROS 2 terminal, listen to the topic:
    ```bash
    ros2 topic echo /unity_chatter
    ```
    You should see the "Hello from Unity!" messages appearing once per second.

## Subscribing to ROS 2 Topics in Unity

Now let's do the reverse: subscribe to a ROS 2 topic to control an object in Unity. We'll make a script that listens for `geometry_msgs/Twist` messages to move a cube.

1.  **Create a Cube**: In Unity, right-click in the Hierarchy > 3D Object > Cube.
2.  **Create a C# Script**: Create a script named `RosSubscriber.cs`.
3.  **Attach and Edit**: Attach the script to the Cube object you just created. Edit the script:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; // Import Twist message

public class RosSubscriber : MonoBehaviour
{
    // ROS Connection
    private ROSConnection ros;
    
    // Topic name
    private string topicName = "/cmd_vel";

    // Movement speed
    public float linearSpeed = 2.0f;
    public float angularSpeed = 180.0f; // degrees per second

    private Vector3 linearVelocity;
    private float angularVelocity;

    void Start()
    {
        // Get the ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to the topic
        ros.Subscribe<TwistMsg>(topicName, ReceiveTwistCommand);
    }

    void Update()
    {
        // Apply the received velocities
        transform.Translate(linearVelocity * Time.deltaTime, Space.World);
        transform.Rotate(0, angularVelocity * Time.deltaTime, 0);
    }

    // Callback function for when a message is received
    void ReceiveTwistCommand(TwistMsg message)
    {
        // Convert ROS coordinate system (right-hand rule, Z-up)
        // to Unity coordinate system (left-hand rule, Y-up)
        
        // ROS linear.x (forward) -> Unity transform.forward (which is local Z)
        // ROS linear.y (left) -> Unity -transform.right (which is local -X)
        // ROS angular.z (yaw) -> Unity rotation around Y axis
        
        linearVelocity = new Vector3((float)message.linear.y, 0, (float)message.linear.x) * linearSpeed;
        angularVelocity = -(float)message.angular.z * angularSpeed;
        
        Debug.Log($"Received Twist: Linear={linearVelocity}, Angular={angularVelocity}");
    }
}
```

**What this script does:**
-   In `Start()`, it subscribes to the `/cmd_vel` topic and registers the `ReceiveTwistCommand` function as the callback.
-   The `ReceiveTwistCommand` function is executed every time a message arrives. It extracts the linear and angular velocities from the `TwistMsg`.
    -   **Coordinate System Conversion is CRITICAL**: Note the conversion from ROS to Unity conventions. This is a common source of bugs.
-   In `Update()`, the script continuously applies the last received velocities to the cube's `transform`, making it move and rotate smoothly.

**To test it:**
1.  Run the `ros_tcp_endpoint` on your ROS 2 machine.
2.  Press Play in Unity.
3.  In your ROS 2 terminal, publish a `Twist` message:
    ```bash
    # Move forward
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
    
    # Turn left
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
    ```
    You should see the cube in your Unity scene move and rotate in response to your commands.

## Important Considerations

-   **Message Types**: The `ROS TCP Connector` automatically generates C# class definitions for standard ROS 2 messages. If you have custom messages in your ROS 2 workspace, you must generate their C# equivalents. You can do this from the Unity Editor menu: **Robotics > Generate ROS Messages...**.
-   **Performance**: While the TCP connector is efficient, sending large amounts of data (like high-resolution images or point clouds) at a high frequency can still cause network latency. Always consider the data rates your application requires.
-   **Error Handling**: In a real application, you would want to add logic to handle disconnections and reconnections to the ROS endpoint. The `ROSConnection` object provides status events you can subscribe to for this purpose.

By mastering publishing and subscribing, you have the complete toolkit to create a "digital twin" in Unity that can both be controlled by and provide sensor data to your ROS 2 algorithms.
