---
---

# Creating Interactive Scenarios in Unity

One of Unity's greatest strengths is its powerful UI framework and component-based architecture, which make it easy to build interactive scenarios for your robot. You can create user interfaces to send commands, display sensor data, or build complex simulations for testing your robot's behavior.

This lesson will cover how to create a simple UI to control your robot and how to set up a basic interactive scenario.

## 1. Building a UI with Unity UI

Let's create a simple UI with a button to send a command to our robot via ROS 2.

**Step 1: Create a Canvas**

All UI elements in Unity must be inside a `Canvas`.
1.  In the Hierarchy window, right-click and select **UI > Canvas**.
2.  This will create a Canvas object and an `EventSystem` object. The `EventSystem` is required to handle input like clicks and drags.
3.  Select the Canvas and in the Inspector, set the **UI Scale Mode** to **Scale With Screen Size**. This makes your UI responsive to different screen resolutions.

**Step 2: Add a Button**

1.  Right-click on your Canvas in the Hierarchy and select **UI > Button - TextMeshPro**.
    -   If it's your first time using TextMeshPro, Unity will prompt you to import essential resources. Click **Import TMP Essentials**.
2.  A button will appear in your scene. You can position it using the `Rect Transform` component in the Inspector.
3.  Expand the Button object in the Hierarchy and select the `Text (TMP)` child object. In its Inspector, change the `Text` field to "Send Command".

**Step 3: Scripting the Button's Action**

Now, let's make the button publish a ROS 2 message when clicked.

1.  Create a new C# script called `UIController.cs`.
2.  Attach this script to your "RosManager" `GameObject` (or any other persistent object).
3.  Edit the script:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UIController : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "ui_commands";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    public void PublishCommand()
    {
        StringMsg message = new StringMsg("Button Clicked!");
        ros.Publish(topicName, message);
        Debug.Log("Sent command: Button Clicked!");
    }
}
```
-   This script is simple: it registers a publisher on `Start()` and has a single public method, `PublishCommand()`, that sends a message.

**Step 4: Linking the Button to the Script**

1.  Select your Button in the Hierarchy.
2.  In the Inspector, find the `Button` component. At the bottom is an `On Click ()` event handler list.
3.  Click the **"+"** icon to add a new event.
4.  Drag your "RosManager" `GameObject` (the one with the `UIController` script) from the Hierarchy into the `None (Object)` field.
5.  Click the dropdown that says "No Function" and navigate to **UIController > PublishCommand()**.

![Button OnClick Event](https://docs.unity3d.com/uploads/Main/UI_Button_OnClick.png)
*(Image credit: Unity Technologies)*

**To test it:**
1.  Run your `ros_tcp_endpoint`.
2.  Press Play in Unity.
3.  In a ROS 2 terminal, run `ros2 topic echo /ui_commands`.
4.  Click the "Send Command" button in your Unity scene. You should see the message appear in your terminal.

## 2. Designing an Interactive Scenario: Pick and Place

A common robotics task is "pick and place." Let's outline how to set up a simple version in Unity. The goal is to have the robot pick up a cube when a ROS 2 command is received.

**Scenario Components:**
1.  **A Robot**: Your imported humanoid or robotic arm, configured with `ArticulationBody` components and a `JointController` script that can receive joint position commands.
2.  **A Target Object**: A simple cube that the robot will "pick up."
3.  **A Trigger Script**: A script that detects when the robot's gripper is close to the cube and "attaches" it.

**Step 1: Create the Target Object**

1.  Create a cube (**Hierarchy > 3D Object > Cube**).
2.  Position it in front of the robot, within reach.
3.  Add a `RigidBody` component to the cube (**Inspector > Add Component > RigidBody**). This allows it to be affected by physics.
4.  On the cube's `Box Collider` component, check the **Is Trigger** box. This turns the collider into a trigger volume, which can detect other colliders entering it without causing a physical collision. This is perfect for detecting when the gripper is near.

**Step 2: Create the Gripper Detector**

1.  On your robot model, identify the gripper link (e.g., `left_gripper_link`).
2.  Add a `Box Collider` to this link and adjust its size to represent the gripping area. Make sure **Is Trigger** is **NOT** checked for this one. We want it to be a physical object.
3.  Add a `RigidBody` to the gripper link as well. Set it to be **Is Kinematic**. This means it won't be affected by physics forces, but it can still register collision events with other objects.

**Step 3: The Interaction Script**

Create a C# script called `PickAndPlace.cs` and attach it to the target cube.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class PickAndPlace : MonoBehaviour
{
    private ROSConnection ros;
    private bool isGripping = false;
    private GameObject gripper;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<BoolMsg>("/gripper/command", GripperStateChange);
    }

    // Callback for ROS 2 gripper command
    void GripperStateChange(BoolMsg message)
    {
        isGripping = message.data;
        Debug.Log($"Gripper state set to: {isGripping}");

        if (!isGripping && gripper != null)
        {
            // Detach the object
            transform.SetParent(null); // Unparent
            GetComponent<Rigidbody>().isKinematic = false; // Re-enable physics
            gripper = null;
        }
    }

    // This function is called when another collider enters this object's trigger
    void OnTriggerStay(Collider other)
    {
        // Check if the gripper is commanded to close and we are near the gripper
        if (isGripping && other.CompareTag("Gripper"))
        {
            // "Pick up" the object
            gripper = other.gameObject;
            transform.SetParent(gripper.transform); // Attach this cube to the gripper
            GetComponent<Rigidbody>().isKinematic = true; // Disable physics
        }
    }
}
```

**What this script does:**
1.  It subscribes to a `/gripper/command` topic, which we assume will be a `std_msgs/Bool` message (`true` to grip, `false` to release).
2.  The `OnTriggerStay` function is a built-in Unity event. It's called every frame that another collider is inside this object's trigger volume.
3.  We check if the other collider has a "Gripper" tag (you should add this tag to your gripper `GameObject` in the Inspector).
4.  If the gripper is present and the `isGripping` flag is true, we "pick up" the cube by **parenting** its transform to the gripper's transform. This makes the cube move along with the gripper. We also set its `RigidBody` to be kinematic to prevent it from falling.
5.  When the `isGripping` flag is set to false, we detach the cube by un-parenting it and re-enabling its physics.

**To test this scenario:**
1.  Move your robot arm so the gripper's collider is intersecting the cube's trigger volume.
2.  Publish a message to command the gripper to close:
    ```bash
    ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: true}"
    ```
3.  The cube should now be parented to the gripper. Move the arm around, and the cube will follow.
4.  Publish a message to command the gripper to open:
    ```bash
    ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: false}"
    ```
5.  The cube should detach and fall.

This simple example demonstrates the power of combining Unity's physics and scripting events with ROS 2 communication to build complex, interactive tests for your robot's software stack.
