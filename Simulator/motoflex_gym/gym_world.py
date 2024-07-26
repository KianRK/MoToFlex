import gymnasium as gym
from gymnasium import spaces
import pygame
import numpy as np
from motoflex_gym import WalkingSimulator
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.pyplot as plt
import PIL.Image
import os
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation
from motoflex_gym.kinematics import *
import random

# https://www.gymlibrary.dev/content/environment_creation/

class MoToFlexEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    # Joints limits for the leg only model

    def __init__(self, 
                 action_function,
                 reward_functions,
                 observation_space,
                 observation_terms,
                 action_space,   
                 random_push=None,
                 render_mode=None,  
                 config_path='config/50.cfg',
                 ab_filter_alpha=None):
        super(MoToFlexEnv, self).__init__()
        self.time = 0
        self.rewards = []
        self.action_data = []
        self.pushes = []
        self.alpha = ab_filter_alpha
        self.action_type = action_function
        self.reward_functions = reward_functions
        self.observation_space = observation_space
        self.observation_terms = observation_terms
        self.action_space = action_space
        self.action_function = action_function
        self.random_push = random_push
        notebook_path = os.path.dirname(os.path.abspath(__file__))
        os.chdir(notebook_path + "/../../")

        # You can change this after creation of the env manually
        self.config_path = config_path

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.last_polar = None

        """
        If human-rendering is used, `self.window` will be a reference
        to the window that we draw to. `self.clock` will be a clock that is used
        to ensure that the environment is rendered at the correct framerate in
        human-mode. They will remain `None` until human-mode is used for the
        first time.
        """
        self.window = None
        self.clock = None
        WalkingSimulator.init()

    def _get_obs(self):
        _obs = self.observation_terms()
        return _obs

    def _get_info(self):
        return {
            "time": self.time
        }

    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed, options=options)
        WalkingSimulator.reset(self.config_path)

        self.last_polar = None
        self.pushes = []

        observation = self._get_obs()
        info = self._get_info()
        

        if self.render_mode == "human":
            self._render_frame()

        self.time = 0

        return observation, info
    
    def _reward(self, last_action):
        obs = self._get_obs()
        self.rewards = []
        for f in self.reward_functions:
            self.rewards.append(f(obs, last_action))

        return (sum(self.rewards))
    
    @staticmethod
    def _random_from_range(range):
        random_force = random.randint(*range)
        random_sign = random.choice([-1, 1])
        random_force *= random_sign
        return random_force

    def step(self, data):
        self.action_data = data

        action, delta_action = self.action_function(data, self)

        if type(action) == np.ndarray:
            action = action.tolist()

        if self.random_push is not None:
            if random.random() < self.random_push["probability"]:
                if "force_range_x" in self.random_push.keys():
                    f_x = MoToFlexEnv._random_from_range(self.random_push["force_range_x"])
                else:
                    f_x = 0.
                if "force_range_y" in self.random_push.keys():
                    f_y = MoToFlexEnv._random_from_range(self.random_push["force_range_y"])
                else:
                    f_y = 0.
                if "force_range_z" in self.random_push.keys():
                    f_z = MoToFlexEnv._random_from_range(self.random_push["force_range_z"])
                else:
                    f_z = 0.
                WalkingSimulator.add_force(0, f_x, f_y, f_z)
                self.pushes.append([f_x, f_y, f_z])

        WalkingSimulator.step(action)

        terminated = self.time == 150

        # Make sure at least one foot has contact to ground
        contact = WalkingSimulator.foot_contact(0) or  WalkingSimulator.foot_contact(1)

        truncated = not WalkingSimulator.is_running() or not contact;
        reward = self._reward(delta_action)
        observation = self._get_obs()
        info = self._get_info()

        if self.render_mode == "human":
            self._render_frame()

        self.time += 1

        return observation, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self):
        if self.window is None and self.render_mode == "human":
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.set_aspect('equal')
            # Achsenbeschriftung
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

        if self.render_mode == "human":
            # The following line copies our drawings from `canvas` to the visible window

            # Zeige das Diagramm
            #plt.show()


            # We need to ensure that human-rendering occurs at the predefined framerate.
            # The following line will automatically add a delay to keep the framerate stable.
            self.clock.tick(self.metadata["render_fps"])
        else:              
            fig = plt.figure()
            canvas = FigureCanvasAgg(fig)

            ax = fig.add_subplot(111, projection='3d')

            ax.set_aspect('equal')
            ax.set_box_aspect([1, 1, 1])

            center = WalkingSimulator.get_6d_pose()

            # Zentriere das gezeichnete Objekt
            ax.auto_scale_xyz([-0.25 + center[0], 0.25 + center[0]], 
                              [-0.25 + center[1], 0.25 + center[1]], 
                              [0, 0.5])

            # Achsenbeschriftung
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            for i in range(WalkingSimulator.get_box_count()):
                pose = WalkingSimulator.get_box_6d_pose(i)
                size = WalkingSimulator.get_box_size(i)
                MoToFlexEnv._draw_cube(ax, size, pose)

            for i in range(WalkingSimulator.get_flexbox_count()):
                for j in range(WalkingSimulator.get_box_count_in_flexbox(i)):
                    pose = WalkingSimulator.get_box_pose_in_flexbox(i, j)
                    size = WalkingSimulator.get_box_size_in_flexbox(i, j)
                    MoToFlexEnv._draw_cube(ax, size, pose)
            
            ax.text2D(0.05, 0.95, "Rew: {} = {:.2f}".format(' + '.join(['{:.2f}'.format(x) for x in self.rewards]), sum(self.rewards)), transform=ax.transAxes)
            ax.text2D(0.05, 0.9, "Action L: {}".format(', '.join(['{:.4f}'.format(x) for x in self.action_data[:len(self.action_data)//2]])), transform=ax.transAxes)
            ax.text2D(0.05, 0.85, "Action R: {}".format(', '.join(['{:.4f}'.format(x) for x in self.action_data[len(self.action_data)//2:]])), transform=ax.transAxes)
            for i, fv in enumerate(self.pushes):
                ax.text2D(0.05, 0.8 - i * 0.05, str(fv), transform=ax.transAxes)

            agg = canvas.switch_backends(FigureCanvasAgg)
            agg.draw()
            img = np.asarray(agg.buffer_rgba())
            rgba_image = PIL.Image.fromarray(img)
            rgb_image = rgba_image.convert('RGB')
            
            ax.clear()
            plt.figure().clear()
            plt.cla()
            plt.clf()
            plt.close('all')
            return np.array(rgb_image)

    def close(self):
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()

    @staticmethod
    def _draw_cube(ax, size, pose):
        # Koordinaten des Würfels
        vertices = [
                    [-size[0]/2, -size[1]/2, -size[2]/2],
                    [size[0]/2, -size[1]/2, -size[2]/2],
                    [size[0]/2, size[1]/2, -size[2]/2],
                    [-size[0]/2, size[1]/2, -size[2]/2],
                    [-size[0]/2, -size[1]/2, -size[2]/2],

                    [-size[0]/2, -size[1]/2, size[2]/2],
                    [size[0]/2, -size[1]/2, size[2]/2],
                    [size[0]/2, size[1]/2, size[2]/2],
                    [-size[0]/2, size[1]/2, size[2]/2],
                    [-size[0]/2, -size[1]/2, size[2]/2],

                    [-size[0]/2, -size[1]/2, -size[2]/2],
                    [size[0]/2, -size[1]/2, -size[2]/2],
                    [size[0]/2, -size[1]/2, size[2]/2],
                    [-size[0]/2, -size[1]/2, size[2]/2],
                    [-size[0]/2, -size[1]/2, -size[2]/2],

                    [-size[0]/2, size[1]/2, -size[2]/2],
                    [size[0]/2, size[1]/2, -size[2]/2],
                    [size[0]/2, size[1]/2, size[2]/2],
                    [-size[0]/2, size[1]/2, size[2]/2],
                    [-size[0]/2, size[1]/2, -size[2]/2],

                    [-size[0]/2, -size[1]/2, -size[2]/2],
                    [-size[0]/2, size[1]/2, -size[2]/2],
                    [-size[0]/2, size[1]/2, size[2]/2],
                    [-size[0]/2, -size[1]/2, size[2]/2],
                    [-size[0]/2, -size[1]/2, -size[2]/2],

                    [size[0]/2, -size[1]/2, -size[2]/2],
                    [size[0]/2, size[1]/2, -size[2]/2],
                    [size[0]/2, size[1]/2, size[2]/2],
                    [size[0]/2, -size[1]/2, size[2]/2],
                    [size[0]/2, -size[1]/2, -size[2]/2],
                    ]

        # Transformation des Würfels basierend auf Position und Orientierung
        rotated_vertices = []
        for vertex in vertices:
            rotated_vertex = MoToFlexEnv._pose_rotation(vertex, pose)
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
        #ax.set_xlim([-size[2], size[2]])
        #ax.set_ylim([-size[2], size[2]])
        #ax.set_zlim([-size[2], size[2]])

    @staticmethod
    def _pose_rotation(point, pose):
        # Extrahiere Position und Orientierung aus dem 6D-Vektor
        position = pose[:3]
        orientation = pose[3:]

        # Konvertiere die Orientierung in eine Rotation
        rotation = Rotation.from_euler("XYZ", orientation)

        # Wende die Rotation auf den Punkt an
        rotated_point = rotation.apply(point)

        # Gib den rotierten Punkt zurück
        return rotated_point
    
    @staticmethod
    def current_polar_pos():
        cur_angles = np.array(WalkingSimulator.get_joint_angles()).astype('float32')
        cur_pose = WalkingSimulator.forward_kinematics([0., *cur_angles.tolist()[0:5], 0., *cur_angles.tolist()[5:]])

        left_cur_polar = cart_to_spherical(*cur_pose[:3])
        right_cur_polar = cart_to_spherical(*cur_pose[6:9])

        return np.concatenate([left_cur_polar, right_cur_polar])

    def delta_polar_to_angles(self, left_drx, left_dry, left_dlen, right_drx, right_dry, right_dlen):
        cur_polar = self.current_polar_pos()

        if self.last_polar is None:
            self.last_polar = cur_polar

        left_new_polar = self.last_polar[:3] + np.array([left_drx, left_dry, left_dlen])
        right_new_polar = self.last_polar[3:] + np.array([right_drx, right_dry, right_dlen])

        if self.alpha is not None:
            self.last_polar = np.concatenate([left_new_polar, right_new_polar])
            alpha = self.alpha
            left_new_polar = alpha * cur_polar[:3] + (1 - alpha) * left_new_polar
            right_new_polar = alpha * cur_polar[3:] + (1 - alpha) * right_new_polar
        else:
            self.last_polar = None

        left_new_cart = spherical_to_cart(*left_new_polar)
        right_new_cart = spherical_to_cart(*right_new_polar)
      
        new_angles = WalkingSimulator.inverse_kinematics([*left_new_cart, 0., 0., 0., 
                                                          *right_new_cart, 0., 0., 0.])

        return new_angles[1:6] + new_angles[7:12]