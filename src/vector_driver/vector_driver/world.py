from anki_vector.util import degrees
import numpy as np
from vector_driver.utils import rotation_matrix_z, rotation_matrix_x, rotation_matrix_y
from enum import Enum
import math

# camera calib: https://www.youtube.com/watch?v=EWqqseIjVqM


class Marker_World:
    
    def __init__(self):
        self.marker_transforms = self.define_marker_world()

    def define_marker_world(self):

        return {
            1139: {  # LEFT
                "pos": np.array([-500, 0, 0]),
                "rot": rotation_matrix_z(90) @ rotation_matrix_x(-90),
                "angle": math.pi
            },
            1140: {   # FRONT
                "pos": np.array([0, 500, 0]),
                "rot": rotation_matrix_x(-90),
                "angle": math.pi /2
            },
            1142: {  # FRONT LEFT
                "pos": np.array([-500, 500, 0]),
                "rot":  rotation_matrix_z(45) @ rotation_matrix_x(-90),
                "angle": (3 *math.pi) / 4
            },
            1145: {   # FRONT RIGHT
                "pos": np.array([500, 500, 0]),
                "rot": rotation_matrix_z(-45) @ rotation_matrix_x(-90),
                "angle": math.pi / 4
            },
            1141: {   # RIGHT 
                "pos": np.array([500, 0, 0]),
                "rot": rotation_matrix_z(-90) @ rotation_matrix_x(-90),
                "angle": 0
            },
            1144: {   # BOTTOM
                "pos": np.array([0, -500, 0]),
                "rot": rotation_matrix_z(180) @ rotation_matrix_x(-90),
                "angle": (3 * math.pi) / 2
            },
            1146: {   # BOTTOM LEFT
                "pos": np.array([-500, -500, 0]),
                "rot": rotation_matrix_z(135) @ rotation_matrix_x(-90),
                "angle": (5 * math.pi) / 2
            },
            1143: {  # BOTTOM RIGHT
                "pos": np.array([500, -500, 0]),
                "rot": rotation_matrix_z(-135) @ rotation_matrix_x(-90),
                "angle": (7* math.pi) / 4
            }
        }
