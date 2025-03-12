import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_vector(ax, origin, vector, color, label):
    ax.quiver(*origin, *vector, color=color, arrow_length_ratio=0.1, label=label)

def visualize_vector_subtraction(v1, v2):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    origin = np.array([0, 0, 0])
    v_diff = v1 - v2
    
    plot_vector(ax, origin, v1, 'r', 'v1')
    plot_vector(ax, origin, v2, 'b', 'v2')
    plot_vector(ax, origin, v_diff, 'g', 'v1 - v2')
    
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([-10, 10])
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    plt.show()

# Example usage:
v1 = np.array([2, 3, 7])
v2 = np.array([2, 3, 6.5])
visualize_vector_subtraction(v1, v2)
