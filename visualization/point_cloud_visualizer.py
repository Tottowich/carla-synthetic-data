from typing import List, Tuple

import numpy as np
import open3d as o3d

import warnings
import open3d

# Define the warning message to filter
warning_message = "GLFW Error: GLX: Failed to make context current"

# Suppress the specific warning message
warnings.filterwarnings("ignore", category=UserWarning, message=warning_message)

import threading

from simulation.sensors.sensor import Sensor


class PointCloudVisualizer:
    """
    This class is used to visualize point clouds in a 3D space.

    Parameters
    ----------
    display_size : Tuple[int, int]
        The size of the display window.
    point_size : int, optional
        The size of the rendered points. Defaults to 1.
    background_color : Tuple[float, float, float, float], optional
        The background color of the display window. Defaults to (0.05, 0.05, 0.05, 0.05).
    """

    def __init__(self, display_size: Tuple[int, int], point_size: int = 1,
                 background_color: Tuple[float, float, float, float] = (0.05, 0.05, 0.05)):
        self.display_size = display_size
        self.display = None
        self.vis = None
        self.init = True

        self.sensor = None
        # Random point cloud

        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(np.random.rand(1000, 3))
        self.pcd.colors = o3d.utility.Vector3dVector(np.random.rand(1000, 3))
        self.point_size = point_size
        self.old_id = None
        self.background_color = background_color

        self.running = True

    def set_options(self, point_size: int = 1,
                    background_color: Tuple[float, float, float, float] = (0.05, 0.05, 0.05, 0.05)):
        """
        Set the rendering options for the point cloud visualization.

        Parameters
        ----------
        point_size : int, optional
            The size of the rendered points. Defaults to 1.
        background_color : Tuple[float, float, float, float], optional
            The background color of the display window. Defaults to (0.05, 0.05, 0.05, 0.05).
        """
        self.point_size = point_size
        self.background_color = background_color
        self.vis.get_render_option().point_size = point_size
        self.vis.get_render_option().background_color = background_color
        self.vis.get_render_option().show_coordinate_frame = True

    def add_sensor(self, sensor: Sensor, position: Tuple[int, int] = None):
        """
        Add a sensor to the point cloud visualizer.

        Parameters
        ----------
        sensor : Sensor
            The sensor to add.
        position : Tuple[int, int], optional
            The position of the sensor in the display. Defaults to None.

        Raises
        ------
        AssertionError
            If sensor is not of type 'Sensor' or is 2D or does not have a 'pcd' attribute.
        """
        assert isinstance(sensor, Sensor), f"Expected sensor to be of type 'Sensor', but got {type(sensor)}"
        assert not sensor.is_2d, f"Expected sensor to be 3D, but got 2D"
        self.sensor = sensor
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(width=self.display_size[0], height=self.display_size[1], left=0, top=0, visible=True,
                               window_name="Open3D")
        # Set full screen
        # self.vis.set_full_screen(True)
        self.set_options(point_size=self.point_size, background_color=self.background_color)
        self.vis.add_geometry(self.pcd)
        # Add coordinate frame
        self.coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        self.vis.add_geometry(self.coord_frame)
        self.vis.poll_events()
        self.vis.update_renderer()
        self.vis.register_key_callback(ord("R"), self._reset_view_callback)
        self.vis.register_key_callback(ord("Q"), self._quit_callback)
        self.vis.register_key_callback(ord("S"), self._start_callback)
    def _reset_view_callback(self, vis):
        """
        Reset the camera view to look at the point cloud.

        Parameters
        ----------
        vis : open3d.visualization.VisualizerWithKeyCallback
            The visualizer object.

        Returns
        -------
        bool
            False.
        """
        vis.reset_view_point(True)
        return False

    def _quit_callback(self, vis):
        """
        Quit the visualization.

        Parameters
        ----------
        vis : open3d.visualization.VisualizerWithKeyCallback
            The visualizer object.

        Returns
        -------
        bool
            False.
        """
        self.running = False # Hello   
        return False

    def _start_callback(self, vis):
        """
        Start the visualization.

        Parameters
        ----------
        vis : open3d.visualization.VisualizerWithKeyCallback
            The visualizer object.

        Returns
        -------
        bool
            False.
        """
        self.running = True
        return False

    def get_display_size(self):
        """
        Get the size of the display window.

        Returns
        -------
        List[int]
            The size of the display window as [width, height].
        """
        return [int(self.display_size[0]), int(self.display_size[1])]

    def update_visualizer(self):
        """Update the point cloud visualizer with the latest point cloud data."""
        scene = self.sensor.get_scene(timeout=1e-3)
        if scene:
            self.pcd.points = scene.points
            self.pcd.colors = scene.colors
            self.vis.update_geometry(self.pcd)
            # Set coordinate frame to the sensor's coordinate frame
            self.vis.poll_events()
            self.vis.update_renderer()
            if self.init:
                self._reset_view_callback(self.vis)
                self.init = False

    def render(self):
        """
        Render the point cloud.

        This method should be called in a loop to continuously update the visualization.

        Note: The rendering only occurs if the visualizer is running.
        """
        if not self.is_running() or self.vis is None:
            return
        self.update_visualizer()

    def destroy(self):
        """Destroy the point cloud visualizer and associated sensor."""
        if self.vis:
            self.vis.destroy_window()
            self.vis = None
            self.display = None

    def render_enabled(self):
        """
        Check if rendering is enabled.

        Returns
        -------
        bool
            True if rendering is enabled, False otherwise.
        """
        return self.vis != None

    def is_running(self):
        """
        Check if the point cloud visualizer is running.

        Returns
        -------
        bool
            True if running, False otherwise.
        """
        return self.running

    def stop(self):
        """Stop the point cloud visualizer."""
        self.running = False
        self.destroy()

