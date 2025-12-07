#!/usr/bin/env python3
"""
Inverse Kinematics Solver Example

Demonstrates IK solving using KDL for a simple 3-DOF arm.
"""

from PyKDL import *
import numpy as np

def create_robot_chain():
    """Create 3-DOF planar robot arm"""
    chain = Chain()

    # Joint 1
    chain.addSegment(Segment(
        Joint(Joint.RotZ),
        Frame(Rotation.Identity(), Vector(0.5, 0, 0))
    ))

    # Joint 2
    chain.addSegment(Segment(
        Joint(Joint.RotZ),
        Frame(Rotation.Identity(), Vector(0.3, 0, 0))
    ))

    # Joint 3
    chain.addSegment(Segment(
        Joint(Joint.RotZ),
        Frame(Rotation.Identity(), Vector(0.2, 0, 0))
    ))

    return chain

def solve_ik(chain, target_x, target_y, target_z=0.0):
    """Solve inverse kinematics"""
    fk_solver = ChainFkSolverPos_recursive(chain)
    ik_solver = ChainIkSolverPos_NR(
        chain,
        fk_solver,
        ChainIkSolverVel_pinv(chain)
    )

    # Target pose
    target = Frame(Rotation.Identity(), Vector(target_x, target_y, target_z))

    # Initial guess
    q_init = JntArray(3)
    q_init[0] = 0.0
    q_init[1] = 0.0
    q_init[2] = 0.0

    # Solve
    q_out = JntArray(3)
    result = ik_solver.CartToJnt(q_init, target, q_out)

    if result >= 0:
        return [q_out[i] for i in range(3)]
    else:
        return None

def verify_solution(chain, joint_angles, target):
    """Verify IK solution with forward kinematics"""
    fk_solver = ChainFkSolverPos_recursive(chain)

    q = JntArray(len(joint_angles))
    for i, angle in enumerate(joint_angles):
        q[i] = angle

    result_frame = Frame()
    fk_solver.JntToCart(q, result_frame)

    achieved = result_frame.p

    error = np.sqrt(
        (target[0] - achieved.x())**2 +
        (target[1] - achieved.y())**2 +
        (target[2] - achieved.z())**2
    )

    return error

def main():
    """Test IK solver"""
    chain = create_robot_chain()

    targets = [
        (0.8, 0.3, 0.0),
        (0.6, 0.4, 0.0),
        (0.5, -0.2, 0.0)
    ]

    for x, y, z in targets:
        print(f"\nTarget: ({x}, {y}, {z})")

        solution = solve_ik(chain, x, y, z)

        if solution:
            print(f"✓ Solution: {[f'{np.degrees(a):.2f}°' for a in solution]}")

            error = verify_solution(chain, solution, (x, y, z))
            print(f"  Error: {error*1000:.2f} mm")
        else:
            print("✗ No solution found")

if __name__ == "__main__":
    main()
