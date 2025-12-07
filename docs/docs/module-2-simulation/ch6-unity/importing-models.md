---
---

# Importing and Configuring Robot Models in Unity

Once you have the Unity Robotics Hub installed, you can bring your robot models to life inside the Unity editor. The `URDF Importer` package is the magic that converts a standard URDF file into a game object that Unity can understand, complete with articulated joints and collision meshes.

## Step 1: Importing a URDF

1.  **Locate Your URDF**: Have your robot's URDF file and its associated mesh files ready. It's best practice to have the meshes in a sub-folder relative to the URDF (e.g., `meshes/`).
2.  **Import into Unity**:
    -   In the Unity Editor, navigate to the **Assets** window.
    -   Right-click and select **Import Robot from URDF**.
    -   A file dialog will open. Select your `.urdf` file.

3.  **Configure the Importer**: An import settings window will appear.
    -   **Axis Type**: Set this to **Z up** (the standard for ROS). Unity's native coordinate system is Y up, so this correctly rotates the model on import.
    -   **Mesh Decomposer**: Keep this on the default setting (`VHACD`) which is good for generating collision meshes.
    -   Click **Import**.

Unity will now process the URDF file, read the links and joints, and create a **prefab**. A prefab is a reusable game object that contains all the components, properties, and child objects that make up your robot.

You will find the new prefab in your `Assets` folder.

![URDF Import Window](https://user-images.githubusercontent.com/25339203/110582823-ce2b4180-812e-11eb-8094-1a9a83ea7447.png)
*(Image credit: Unity Technologies)*

## Step 2: Examining the Imported Robot

Drag the newly created robot prefab from your `Assets` folder into the **Hierarchy** window or directly into the **Scene** view.

Now, select the robot in the Hierarchy. Look at the **Inspector** window. You will see several important components that have been automatically added:

-   **`Articulation Body`**: This is the core physics component for robotics in Unity.
    -   The **root link** of the robot (the one with no parent joint) will have a main `Articulation Body` component.
    -   Every **child link** will also have an `Articulation Body` component, configured to represent its joint connection to its parent.
-   **`URDF Link`**: A script that holds information from the original URDF link, including its name.
-   **`URDF Joint`**: A script attached to each child link that defines the joint type (`Revolute`, `Prismatic`, `Fixed`, etc.), motion limits, and other properties.

### Understanding Articulation Bodies

The `Articulation Body` component is what makes a multi-jointed robot behave correctly. Unlike standard `RigidBody` physics in Unity, which is designed for stacking boxes and simple collisions, articulations are designed for the chain-like dependencies of a robot arm or leg.

Key properties of an `Articulation Body` on a joint:
-   **Joint Type**: `Revolute`, `Prismatic`, etc.
-   **Motion**: `Locked`, `Limited`, or `Free`. For a revolute joint with limits, this would be `Limited`.
-   **Lower and Upper Limits**: The joint's range of motion in degrees or meters.
-   **Stiffness and Damping**: These parameters control the "springiness" of the joint motor. You can set a `Target` position and use these values to drive the joint.

![Articulation Body Component](https://docs.unity3d.com/2022.3/Documentation/uploads/Main/Inspector-ArticulationBody.png)
*(Image credit: Unity Technologies)*

## Step 3: Configuring the Joints

The URDF importer does a good job, but you often need to fine-tune the joint settings.

1.  **Select a Joint**: In the Hierarchy, expand the robot's object tree and select a link that has a joint (e.g., `shoulder_link`).
2.  **Inspect the Articulation Body**:
    -   Verify that the **Joint Type** is correct.
    -   Set the **Motion** to `Limited`.
    -   Enter the correct **Lower** and **Upper Limits** in degrees. These should match the safety limits of your physical robot.
    -   Add a small amount of **Damping** (e.g., 10-100) to improve stability and prevent uncontrolled oscillations.

## Step 4: Adding Controllers

To make the robot move, you need to add a controller script that can interface with the `Articulation Body` components and with ROS 2.

The Robotics Hub provides a helpful script called `AGVController` for simple wheeled robots, but for a humanoid, you'll typically write a custom script.

Let's look at the logic for a basic joint controller script.

1.  **Create a C# Script**: In your `Assets` folder, right-click, select **Create > C# Script**, and name it `JointController`.
2.  **Add the script to your robot**: Drag the script onto the root object of your robot prefab in the Hierarchy.
3.  **Edit the script**: Double-click the script to open it in Visual Studio or your code editor of choice.

Here is a simplified example of a script that subscribes to a ROS 2 `Float64MultiArray` topic for joint commands and applies them to the robot's joints.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class JointController : MonoBehaviour
{
    // A dictionary to hold our joint objects
    private System.Collections.Generic.Dictionary<string, ArticulationBody> jointArticulationBodies;

    // ROS Connector
    private ROSConnection ros;
    
    // Start is called before the first frame update
    void Start()
    {
        // Get the ROS connection instance
        ros = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to the joint command topic
        ros.Subscribe<Float64MultiArrayMsg>("/joint_commands", ReceiveJointCommands);

        // --- Find and map all the joints ---
        jointArticulationBodies = new System.Collections.Generic.Dictionary<string, ArticulationBody>();
        
        // Find all ArticulationBody components in the robot
        ArticulationBody[] allJoints = GetComponentsInChildren<ArticulationBody>();
        
        foreach (ArticulationBody joint in allJoints)
        {
            // The root link has no parent, so we skip it
            if (joint.isRoot) continue;
            
            // Get the URDFJoint script to find the original joint name
            URDFJoint urdfJoint = joint.GetComponent<URDFJoint>();
            if (urdfJoint != null)
            {
                jointArticulationBodies.Add(urdfJoint.jointName, joint);
            }
        }
    }

    // Callback function for when we receive a joint command message
    void ReceiveJointCommands(Float64MultiArrayMsg message)
    {
        // Assuming the message data array is ordered correctly
        // and matches the number of controllable joints.
        
        // A more robust implementation would include the joint names in the message.
        int i = 0;
        foreach (var joint in jointArticulationBodies)
        {
            if (i < message.data.Length)
            {
                // Get the target position from the message
                float targetPosition = (float)message.data[i];
                
                // Get the ArticulationBody for this joint
                ArticulationBody articulationBody = joint.Value;
                
                // Set the target for the joint's drive
                var drive = articulationBody.xDrive;
                drive.target = targetPosition;
                articulationBody.xDrive = drive;
            }
            i++;
        }
    }
}
```

This script does the following:
1.  On `Start`, it gets the `ROSConnection` and subscribes to a topic named `/joint_commands`.
2.  It finds all `ArticulationBody` components on the robot and stores them in a dictionary, mapped by their URDF joint name.
3.  When a message is received on `/joint_commands`, the `ReceiveJointCommands` function is called.
4.  It iterates through the message data and applies each value as a `target` for the corresponding joint's `xDrive`. The articulation body's internal physics motor will then work to move the joint to that target position.

By following these steps, you can successfully import any URDF-based robot, configure its physics properties, and add a basic ROS 2 controller to bring it to life in a stunning Unity environment.
