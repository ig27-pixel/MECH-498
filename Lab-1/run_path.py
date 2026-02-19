"""
MECH 498 Lab 1 Part 8c
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import yaml
import lab1_utility as util
import lab1

def load_path(filename):
    """Load joint angles from yaml file"""
    with open(filename, 'r') as f:
        data = yaml.safe_load(f)
    
    # Convert to array
    path = []
    for i in range(len(data["j1"])):
        path.append([data["j1"][i], data["j2"][i], data["j3"][i],
                     data["j4"][i], data["j5"][i], data["j6"][i]])
    
    return np.array(path).T  # Transpose so each column is a time step

# Load the path
path_data = load_path('path.yaml')
n_frames = path_data.shape[1]

# Only use every 5th frame to keep file size reasonable
step = 5
frames = list(range(0, n_frames, step))

# Setup figure
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Track end effector position
trajectory = []

def animate(i):
    """Update function for animation"""
    frame_num = frames[i]
    
    # Clear and reset view
    ax.clear()
    ax.set_xlim([-200, 400])
    ax.set_ylim([-200, 200])
    ax.set_zlim([-100, 300])
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.view_init(elev=22.8, azim=147.3)
    
    # Get angles for this frame
    actuator_angles = path_data[0:3, frame_num]
    gimbal_angles = path_data[3:6, frame_num]
    
    # Draw the robot
    util.draw_phantom(actuator_angles, gimbal_angles, ax)
    
    # Track end effector
    joint_angles = lab1.actuator_to_joint(actuator_angles)
    T, _ = lab1.phantom_fk(joint_angles, gimbal_angles)
    trajectory.append(T[0:3, 3])
    
    # Draw trajectory
    if len(trajectory) > 1:
        traj = np.array(trajectory)
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'b-', linewidth=2)
    
    ax.set_title(f'Frame {i+1}/{len(frames)}')

# Create animation
anim = FuncAnimation(fig, animate, frames=len(frames), interval=50, repeat=True)

# Save as GIF
writer = PillowWriter(fps=20)
anim.save('phantom_video.gif', writer=writer, dpi=80)
