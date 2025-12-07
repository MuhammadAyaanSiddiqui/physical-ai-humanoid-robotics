---
---

# Tuning Physics Parameters in Gazebo

A simulation is only as good as its physics. To create realistic and stable simulations, you must understand how to tune Gazebo's physics parameters. These settings control everything from how objects fall to how they interact upon collision.

Physics parameters can be set at two levels:
1.  **Globally** in the World (`.sdf`) file.
2.  **Locally** on a per-model basis within the model's SDF or URDF file.

## 1. Global Physics Parameters (in the World File)

Global parameters define the baseline physics for the entire simulation. They are configured within the `<physics>` tag of your world file.

```xml
<sdf version="1.7">
  <world name="default">
    ...
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
      <solver>
        <ode>
          <solver>
            <type>quick</type>
            <iters>50</iters>
            <sor>1.3</sor>
          </solver>
        </ode>
      </solver>
      <constraints>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
      </constraints>
    </physics>
    ...
  </world>
</sdf>
```

### Key Global Parameters:

-   **`<gravity>`**: A 3D vector defining the gravitational force. The default `0 0 -9.81` simulates Earth's gravity. Change this to simulate other planets or zero-gravity environments.

-   **`<max_step_size>`**: The duration of each physics update step in seconds. This is a critical parameter for stability.
    -   **Smaller values** (`0.001` or less) lead to higher accuracy and stability but require more computation.
    -   **Larger values** (`0.01` or more) can make the simulation faster but may cause instability, jittering, or objects passing through each other.
    -   **Rule of thumb**: `max_step_size` should be `1 / (10 * max_frequency)` of your system. For a 100 Hz robot controller, you'd want a step size of at least `0.001`.

-   **`<real_time_update_rate>`**: How often Gazebo tries to update the simulation state per second. `1000` Hz (matching a `max_step_size` of `0.001`) is common. If the simulation cannot keep up, the `real_time_factor` will drop below 1.

-   **`<real_time_factor>` (RTF)**: The ratio of simulation time to real time.
    -   `RTF = 1`: The simulation runs in real-time.
    -   `RTF < 1`: The simulation is running slower than real-time (computationally intensive).
    -   `RTF > 1`: The simulation is running faster than real-time.

-   **`<solver>` and `<constraints>`**: These are advanced settings that determine how the physics engine resolves collisions and joint limits.
    -   **`iters`**: The number of iterations the solver runs for each step. More iterations improve accuracy for complex contact scenarios but are slower.
    -   **`sor`**: Successive Over-Relaxation, a parameter for the `quick` solver.
    -   **`cfm`**: Constraint Force Mixing. A small non-zero value can add compliance (softness) to joints and contacts, which can improve stability.
    -   **`erp`**: Error Reduction Parameter. Helps correct joint errors.

## 2. Local Physics Parameters (in the Model File)

Local parameters are defined for each individual link and joint, allowing you to create objects with diverse physical properties.

### Inertial Properties (`<inertial>`)

This is the most important property for dynamic simulation. It defines a link's mass and its resistance to rotation (moment of inertia).

```xml
<link name="my_link">
  <inertial>
    <mass>5.0</mass>  <!-- Mass in kilograms -->
    <inertia>
      <!-- Moment of inertia matrix -->
      <ixx>0.01</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.02</iyy>
      <iyz>0.0</iyz>
      <izz>0.03</izz>
    </inertia>
  </inertial>
  ...
</link>
```

**Tuning Inertia:**
-   **Incorrect Mass**: If a robot link's mass is too low, it may appear "bouncy" or be easily knocked around. If it's too high, motors may struggle to move it.
-   **Incorrect Inertia Matrix**: This is harder to intuit, but an incorrect inertia matrix will cause the link to rotate unnaturally when forces are applied. CAD software (like SolidWorks or Fusion 360) can often calculate these values for you based on the model's geometry and material density. Online calculators are also available for simple shapes.
-   **Debugging Tip**: If your robot is unstable, start by increasing the mass and inertia of the base link. A stable base can solve many problems.

### Contact & Friction Properties (`<collision>` and `<surface>`)

These properties define how a link interacts with other objects upon contact. They are defined within the `<collision>` tag of a link.

```xml
<collision name="my_collision">
  <geometry>...</geometry>
  <surface>
    <contact>
      <ode>
        <kp>1e6</kp>   <!-- Stiffness -->
        <kd>100</kd>   <!-- Damping -->
        <min_depth>0.001</min_depth>
        <max_vel>1.0</max_vel>
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>0.8</mu>    <!-- Coefficient of static friction -->
        <mu2>0.8</mu2>   <!-- Coefficient of static friction (secondary direction) -->
      </ode>
    </friction>
  </surface>
</collision>
```

**Key Surface Parameters:**

-   **`<mu>` and `<mu2>` (Friction Coefficients)**: Define the force needed to make an object slide.
    -   `0.0`: A frictionless surface, like ice.
    -   `1.0`: A high-friction surface, like rubber on asphalt.
    -   Increase `mu` for robot feet or wheels to prevent slipping. Decrease it for objects you want to slide easily.

-   **`<kp>` (Stiffness)** and **`<kd>` (Damping)**: These define the "springiness" of a contact.
    -   `kp` is the spring stiffness. Higher values make the contact harder.
    -   `kd` is the damping. Higher values dissipate energy faster, reducing bouncing.
    -   **Tuning Tip**: If objects are too "bouncy," increase `kd`. If they seem to pass through each other, increase `kp`. Finding the right balance is key to stability.

-   **`<min_depth>`**: The allowable penetration between surfaces before a corrective force is applied. A small value can help stabilize contacts.

### Joint Properties (`<joint>`)

Joint properties control the physics of the connections between links.

```xml
<joint name="my_joint" type="revolute">
  ...
  <physics>
    <ode>
      <limit>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
      </limit>
      <damping>0.1</damping> <!-- Joint damping -->
    </ode>
  </physics>
</joint>
```

-   **`<damping>`**: Simulates friction within the joint.
    -   A small amount of damping (`0.1` to `0.5`) is often realistic and can significantly improve the stability of a robot by preventing uncontrolled oscillations.
    -   If a robot arm swings wildly after a movement, try increasing the damping on its joints.

## A Practical Tuning Workflow

1.  **Start with the World**: Set a reasonable `max_step_size` (e.g., `0.001`) and Earth gravity.
2.  **Get Mass and Inertia Right**: Use CAD tools or calculators to get realistic inertial values for each link. When in doubt, overestimate slightly for stability.
3.  **Tune Friction**: Set the friction for the robot's feet/wheels to a high value (e.g., `0.7` - `1.0`) to ensure good grip.
4.  **Add Joint Damping**: Add a small amount of damping to all non-fixed joints to reduce oscillations.
5.  **Address Instability**:
    -   **Jittering?** Your `max_step_size` might be too large, or `kp`/`kd` values for contacts are too high.
    -   **Bouncing?** Increase the `<kd>` (damping) on the colliding surfaces.
    -   **Passing through objects?** Increase `<kp>` (stiffness) or decrease `max_step_size`.
    -   **Slipping?** Increase the friction coefficient `mu`.

Tuning is an iterative process. Make small changes, observe the results, and build an intuition for how each parameter affects the simulation. A well-tuned simulation is not only more realistic but also more stable and reliable for developing and testing your robotics algorithms.
