import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_aspect('equal')

def draw_cube(ax, size, pose):

    # Koordinaten des Würfels
    vertices = [
                [-size/2, -size/2, -size/2],
                [size/2, -size/2, -size/2],
                [size/2, size/2, -size/2],
                [-size/2, size/2, -size/2],
                [-size/2, -size/2, -size/2],

                [-size/2, -size/2, size/2],
                [size/2, -size/2, size/2],
                [size/2, size/2, size/2],
                [-size/2, size/2, size/2],
                [-size/2, -size/2, size/2],

                [-size/2, -size/2, -size/2],
                [size/2, -size/2, -size/2],
                [size/2, -size/2, size/2],
                [-size/2, -size/2, size/2],
                [-size/2, -size/2, -size/2],

                [-size/2, size/2, -size/2],
                [size/2, size/2, -size/2],
                [size/2, size/2, size/2],
                [-size/2, size/2, size/2],
                [-size/2, size/2, -size/2],

                [-size/2, -size/2, -size/2],
                [-size/2, size/2, -size/2],
                [-size/2, size/2, size/2],
                [-size/2, -size/2, size/2],
                [-size/2, -size/2, -size/2],

                [size/2, -size/2, -size/2],
                [size/2, size/2, -size/2],
                [size/2, size/2, size/2],
                [size/2, -size/2, size/2],
                [size/2, -size/2, -size/2],

                

                ]

    # Transformation des Würfels basierend auf Position und Orientierung
    rotated_vertices = []
    for vertex in vertices:
        rotated_vertex = pose_rotation(vertex, pose)
        translated_vertex = [rotated_vertex[0] + pose[0],
                             rotated_vertex[1] + pose[1],
                             rotated_vertex[2] + pose[2]]
        rotated_vertices.append(translated_vertex)

    # Erzeuge Poly3DCollection für den Würfel
    cube = Poly3DCollection([rotated_vertices])
    cube.set_edgecolor('k')
    cube.set_alpha(0.2)
    ax.add_collection3d(cube)

    # Achsenbeschriftung
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Zeige das Diagramm
    ax.set_xlim([-size, size])
    ax.set_ylim([-size, size])
    ax.set_zlim([-size, size])

def pose_rotation(point, pose):
    # Extrahiere Position und Orientierung aus dem 6D-Vektor
    position = pose[:3]
    orientation = pose[3:]

    # Konvertiere die Orientierung in eine Rotation
    rotation = Rotation.from_euler("XYZ", orientation)

    # Wende die Rotation auf den Punkt an
    rotated_point = rotation.apply(point)

    # Gib den rotierten Punkt zurück
    return rotated_point

draw_cube(ax, 1, [0, 0, 0, 0.78, 0, 0])

# Achsenbeschriftung
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Zeige das Diagramm
plt.show()