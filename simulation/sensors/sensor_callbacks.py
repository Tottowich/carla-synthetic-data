import os
import sys
import time
import yaml
from typing import List, Union, Tuple

import carla
import cv2
import numpy as np
import open3d as o3d

from simulation.environment.carla_actor import CarlaActor
from simulation.sensors.cameras import Camera
from simulation.sensors.lidars import LiDAR_Sensor
from simulation.sensors.sensor import Sensor, SensorCallback
from utils.bounding_boxes import get_image_point, world2sensor
from utils.helper import carla_vec2np_array, deg2rad
from utils.matrix_transforms import rotation2euler
from utils.carla_objects import CarlaObject


class BoundingBoxProjector(SensorCallback):
    """
    This class is used to visualize the bounding boxes of the provided classes.
    """
    def __init__(self, object_list: List[CarlaActor] = None, *args, **kwargs):
        """
        Initialize a BoundingBoxProjector object.

        Parameters
        ----------
        object_list : List[CarlaActor], optional
            The list of objects to visualize.

        Raises
        ------
        AssertionError
            If object_list is not of type 'List[CarlaActor]'.
        """
        super().__init__(*args, **kwargs)
        assert isinstance(object_list, List), f"Expected object_list to be of type 'List[CarlaActor]', but got {type(object_list)}"
        assert all([isinstance(obj, CarlaActor) for obj in object_list]), f"Expected object_list to be of type 'List[CarlaActor]', but got {type(object_list)}"
        
        self.object_list = object_list
        self.cos_threshold = None
        self.sensor_range = None
        self.edges = np.array([[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]])

    def _add_sensor(self, sensor: Camera):
        """
        Add a sensor to the BoundingBoxProjector.

        Parameters
        ----------
        sensor : Camera
            The camera sensor to be added.

        Raises
        ------
        AssertionError
            If sensor is not of type 'Camera'.
        """
        assert isinstance(sensor, Camera), f"Expected sensor to be of type 'Camera', but got {type(sensor)}"
        self.cos_threshold = np.cos(deg2rad(self.sensor.fov/2))
        self.sensor_range = sensor.sensor_range

    def _remove_sensor(self, sensor: Sensor):
        """
        Remove a sensor from the BoundingBoxProjector.

        Parameters
        ----------
        sensor : Sensor
            The sensor to be removed.
        """
        self.object_list = None
        self.cos_threshold = None

    def callback(self, image: np.ndarray):
        """
        Callback function to render bounding boxes on an image.

        Parameters
        ----------
        image : np.ndarray
            The image to render the bounding boxes on.

        Returns
        -------
        np.ndarray
            The image with rendered bounding boxes.

        Raises
        ------
        AssertionError
            If BoundingBoxProjector has not been added to a sensor yet.
        """
        assert self.sensor is not None, f"{self.name} has not been added to a sensor yet"
        # Get camera matrix
        K = self.sensor.K
        location = self.sensor.location()
        forward_direction = self.sensor.forward_vector(normalized=True)
        w2s = self.sensor.w2s(numpy=True)

        for obj in self.object_list:
            bbox = obj.bounding_box
            bbox = self.extend_bbox(bbox)
            color = obj.label.color.astype(np.uint8)
            # Filter bounding boxes that are too far away
            if not self.filter_bbox(bbox, location, forward_direction):
                continue
            verts = [v for v in bbox.get_world_vertices(carla.Transform())]
            # Project the vertices to the image plane

            for edge in self.edges:
                p1 = get_image_point(verts[edge[0]], K, w2s)
                p2 = get_image_point(verts[edge[1]], K, w2s)

                cv2.line(image, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (int(color[0]), int(color[1]), int(color[2])), 1)
        return image
    
    def filter_bbox(self, bbox: carla.BoundingBox, location: carla.Location, forward_direction: carla.Vector3D)->bool:
        """
        Filter bounding boxes that are too far away or not in the field of view.

        Parameters
        ----------
        bbox : carla.BoundingBox
            The bounding box to filter.
        location : carla.Location
            The location of the sensor.
        forward_direction : carla.Vector3D
            The forward direction of the sensor.

        Returns
        -------
        bool
            True if the bounding box is in the field of view and not too far away, False otherwise.
        """
        # Filter bounding boxes that are too far away
        if bbox.location.distance(location) > self.sensor_range:
            return False
        ray = bbox.location - location
        ray_direction = ray/ray.length()

        # Filter bounding boxes that are not in the field of view
        cos_sim = forward_direction.dot(ray_direction)
        if cos_sim < self.cos_threshold:
            return False
        return True
    def extend_bbox(self,bbox:carla.BoundingBox, min_extent:float=0.05):
        """
        Extend the bounding box such that it can be used for visualization.
        
        Parameters
        ----------
        bbox : carla.BoundingBox
            The bounding box to extend.
        
        Returns
        -------
        carla.BoundingBox
            The extended bounding box.
        """
        bbox.extent.x = max(bbox.extent.x, min_extent)
        bbox.extent.y = max(bbox.extent.y, min_extent)
        bbox.extent.z = max(bbox.extent.z, min_extent)
        return bbox

class BoundingBoxOriented(BoundingBoxProjector):
    """
    This class is used to visualize the bounding boxes of the provided classes.    
    """
    def __init__(self, object_list: List[CarlaActor] = None, visualize:bool=True,*args, **kwargs):
        self.visualize = visualize
        self.vis: o3d.visualization.Visualizer = None
        self.visible_bboxes = []
        self.object_box_pairs:List[Tuple(o3d.geometry.OrientedBoundingBox,CarlaObject)] = []
        super().__init__(object_list, *args, **kwargs)
    def _add_sensor(self, sensor: LiDAR_Sensor):
        """
        Add a sensor to the BoundingBoxProjector.

        Parameters
        ----------
        sensor : LiDAR_Sensors
            The camera sensor to be added.

        Raises
        ------
        AssertionError
            If sensor is not of type 'LiDAR_Sensor'.
        """
        assert isinstance(sensor, LiDAR_Sensor), f"Expected sensor to be of type 'Camera', but got {type(sensor)}"
        assert isinstance(self.sensor, LiDAR_Sensor), f"Expected sensor to be of type 'LiDAR_Sensor', but got {type(sensor)}"
        self.cos_threshold = np.cos(deg2rad(self.sensor.fov/2))
        self.sensor_range = sensor.sensor_range
        self.bboxes_oriented = self.initialize_bboxes()
        if self.visualize:
            self.initialize_visualizer()

    
    def initialize_bboxes(self):
        """
        Initialize the bounding boxes.

        Returns
        -------
        List[o3d.geometry.OrientedBoundingBox]
            The list of bounding boxes.
        """
        bbox_set = []
        for obj in self.object_list:
            bbox = obj.bounding_box
            bbox = self.extend_bbox(bbox)

            verts = [v for v in bbox.get_world_vertices(carla.Transform())]
            verts = np.array([carla_vec2np_array(v) for v in verts])
            bbox = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(verts))
            bbox.color = obj.label.color/255
            bbox_set.append(bbox)
            self.hide_bbox(bbox)
            self.object_box_pairs.append((obj, bbox))
        return bbox_set
    
    def initialize_visualizer(self):
        """
        Initialize the visualizer.
        """
        self.vis = self.sensor.display_manager.vis
        for bbox in self.bboxes_oriented:
            self.vis.add_geometry(bbox)
    
    def hide_bbox(self, bbox: o3d.geometry.OrientedBoundingBox):
        """
        Hide a bounding box.

        Parameters
        ----------
        bbox : o3d.geometry.OrientedBoundingBox
            The bounding box to hide.
        """
        bbox.extent = np.array([0, 0, 0])
    def callback(self, scene: o3d.geometry.PointCloud):
        """
        Callback function to render bounding boxes in a point cloud.

        Parameters
        ----------
        scene : o3d.geometry.PointCloud
            The cloud to add the bounding boxes to.

        Returns
        -------
        o3d.geometry.PointCloud
            The cloud with rendered bounding boxes.

        Raises
        ------
        AssertionError
            If BoundingBoxOriented has not been added to a sensor yet.
        """
        assert self.sensor is not None, f"{self.name} has not been added to a sensor yet"
        location = self.sensor.location()
        forward_direction = self.sensor.forward_vector(normalized=True)
        w2s = self.sensor.w2s(numpy=True)
        self.visible_bboxes = []
        for obj, bbox_oriented in self.object_box_pairs:
            bbox = obj.bounding_box
            bbox = self.extend_bbox(bbox)
            # Filter bounding boxes that are too far away
            if not self.filter_bbox(bbox, location, forward_direction, scene):
                self.hide_bbox(bbox_oriented)
                if self.visualize:
                    self.vis.update_geometry(bbox_oriented)
                continue
            verts = self.bbox2verts(bbox, carla.Transform())
            verts = world2sensor(verts, w2s)

            # Negate the y-axis to match the camera coordinate frame
            verts[:, 1] *= -1
            # Create a new bounding box from the vertices
            bbox_oriented_copy = bbox_oriented.create_from_points(o3d.utility.Vector3dVector(verts))
            visibility = self._check_visible(bbox_oriented_copy, scene)
            if not visibility:
                self.hide_bbox(bbox_oriented)
                if self.visualize:
                    self.vis.update_geometry(bbox_oriented)
                continue
            # Update the bounding box with the new values.
            # This is necessary because the create_from_points function does not update the bounding box inplace.
            bbox_oriented.extent = bbox_oriented_copy.extent
            bbox_oriented.center = bbox_oriented_copy.center
            bbox_oriented.R = bbox_oriented_copy.R

            self.visible_bboxes.append((bbox_oriented, obj.label))
            if self.visualize:
                self.vis.update_geometry(bbox_oriented)
        return scene
    
    def bbox2verts(self, bbox: carla.BoundingBox, transform: carla.Transform):
        """
        Get the vertices of a bounding box.

        Parameters
        ----------
        bbox : carla.BoundingBox
            The bounding box to get the vertices from.
        transform : carla.Transform
            The transform of the bounding box.

        Returns
        -------
        np.ndarray
            The vertices of the bounding box.
        """
        verts = [v for v in bbox.get_world_vertices(transform)]
        verts = np.array([carla_vec2np_array(v) for v in verts])
        return verts
    
    def filter_bbox(self, bbox: carla.BoundingBox, location: carla.Location, forward_direction: carla.Vector3D, scene: o3d.geometry.PointCloud)->bool:
        """
        Filter bounding boxes that are too far away or not in the field of view.

        Parameters
        ----------
        bbox : carla.BoundingBox
            The bounding box to filter.
        location : carla.Location
            The location of the sensor.
        forward_direction : carla.Vector3D
            The forward direction of the sensor.

        Returns
        -------
        bool
            True if the bounding box is in the field of view and not too far away, False otherwise.
        """
        # Filter bounding boxes that are too far away
        if bbox.location.distance(location) > self.sensor_range:
            return False
        ray = bbox.location - location
        ray_direction = ray/ray.length()

        # Filter bounding boxes that are not in the field of view
        cos_sim = forward_direction.dot(ray_direction)
        if cos_sim < self.cos_threshold:
            return False
        return True
    def _check_visible(self, bbox: carla.BoundingBox, scene: o3d.geometry.PointCloud) -> bool:
        """
        Check if a bounding box is visible in a point cloud.

        Parameters
        ----------
        bbox : carla.BoundingBox
            The bounding box to check.
        scene : o3d.geometry.PointCloud
            The point cloud to check.

        Returns
        -------
        bool
            True if the bounding box is visible in the point cloud, False otherwise.
        """
        points_inside_bbox = bbox.get_point_indices_within_bounding_box(scene.points)
        return len(points_inside_bbox) > 0
class BoundingBoxMultiSensor(BoundingBoxOriented):
    """
    This class is used to visualize the bounding boxes of the provided classes.    
    """
    def filter_bbox(self, bbox: carla.BoundingBox, location: carla.Location, forward_direction: carla.Vector3D, scene: o3d.geometry.PointCloud)->bool:
        """
        Filter bounding boxes that are too far away or not in the field of view.

        Parameters
        ----------
        bbox : carla.BoundingBox
            The bounding box to filter.
        location : carla.Location
            The location of the sensor.
        forward_direction : carla.Vector3D
            The forward direction of the sensor.
        

        Returns
        -------
        bool
            True if the bounding box is in the field of view and not too far away, False otherwise.
        """
        # Filter bounding boxes that are too far away
        for i, (loc, sensor_range, forward) in enumerate(zip(location, self.sensor_range, forward_direction)):
            if bbox.location.distance(loc) > sensor_range:
                continue
            ray = bbox.location - loc
            ray_direction = ray/ray.length()

            # Filter bounding boxes that are not in the field of view
            cos_sim = forward.dot(ray_direction)
            if cos_sim < self.cos_threshold[i]:
                continue

            return True # if one sensor sees the object, return True
        return False # if no sensor sees the object, return False
    

class SceneExporter(BoundingBoxMultiSensor):
    """
    Instead of rendering the bounding boxes this class exports them along with corresponding point clouds.
    Each scene is saved in a separate .npy file. The labels are saved in a separate .txt file with the same name.
    
    Structure of the .npy file:
    [x, y, z, intensity]
    Structure of the .txt file:
    [x, y, z, dx, dy, dz, yaw, label]

    Structure of the folder:

    base_folder_output

    ├── points
    │   ├── 000000.npy
    │   ├── 000001.npy
    │   ├── ...
    ├── labels
    │   ├── 000000.txt
    │   ├── 000001.txt
    │   ├── ...
    ├── data_info.yaml
    optionally:

    base_folder_output
    ├── points
    │   ├── 000000.npy
    │   ├── 000001.npy
    │   ├── ...
    ├── labels
    │   ├── 000000.txt
    │   ├── 000001.txt
    │   ├── ...
    ├── images
    │   ├── 000000.png
    │   ├── 000001.png
    │   ├── ...
    ├── data_info.yaml




    Parameters
    ----------
    object_list : List[CarlaActor], optional
        The list of objects to export.
    visualize : bool, optional
        Whether to visualize the bounding boxes or not. If True the sensor must be added to a DisplayManager.
    base_folder_output : str, optional
        The base folder for saving the data. Defaults to './data'.
    """
    def __init__(self, object_list = List[CarlaActor], visualize:bool=False, base_folder_output:str='./data',*args, **kwargs):
        super().__init__(object_list, visualize,*args, **kwargs)
        self.base_folder_output = base_folder_output
        self.points_folder = None
        self.labels_folder = None
        self.data_info_path = None
        self.data_info = None

        self.frame_number = 0
        self.name_template = '{:06d}'

    def _add_sensor(self, sensor: LiDAR_Sensor):
        """
        Add a sensor to the BoundingBoxProjector.

        Parameters
        ----------
        sensor : LiDAR_Sensors
            The camera sensor to be added.

        Raises
        ------
        AssertionError
            If sensor is not of type 'LiDAR_Sensor'.
        """
        super()._add_sensor(sensor)
        self.create_folders()
        self.create_data_info()

    def create_folders(self):
        """
        Create the folders for saving the data.
        """
        self.points_folder = os.path.join(self.base_folder_output, 'points')
        if os.path.exists(self.points_folder):
            # Set the frame number to the number of files in the folder + 1 to avoid overwriting existing files
            self.frame_number = len(os.listdir(self.points_folder)) + 1
        os.makedirs(self.points_folder, exist_ok=True)
        self.labels_folder = os.path.join(self.base_folder_output, 'labels')
        os.makedirs(self.labels_folder, exist_ok=True)


    def create_data_info(self):
        """
        Create the data info file.
        """
        label_names = list(set([obj.label.name for obj in self.object_list]))
        self.data_info = {
            'points_folder': self.points_folder,
            'labels_folder': self.labels_folder,
            'label_names': label_names,
            'sensor': self.sensor.name,
            'sensor_fov': self.sensor.fov.tolist(),
            'sensor_channels': self.sensor.channels.tolist(),
            'sensor_range': self.sensor.sensor_range.tolist(),
            'sensor_points_per_second': self.sensor.pps.tolist(),
            'sensor_rotation_frequency': self.sensor.rotation_frequency.tolist(),
            'sensor_lower_fov': self.sensor.lower_fov.tolist(),
            'sensor_upper_fov': self.sensor.upper_fov.tolist(),
        }
        self.data_info_path = os.path.join(self.base_folder_output, 'data_info.yaml')
        with open(self.data_info_path, 'w') as f:
            yaml.dump(self.data_info, f,sort_keys=False)
    
    def callback(self, scene: o3d.geometry.PointCloud):
        """
        Callback function to export bounding boxes and point clouds.

        Parameters
        ----------
        scene : o3d.geometry.PointCloud
            The cloud to add the bounding boxes to and to export.
        
        Returns
        -------
        o3d.geometry.PointCloud
            The cloud with rendered bounding boxes.
        """
        super().callback(scene)
        self.export_scene(scene)

    def export_scene(self, scene: o3d.geometry.PointCloud):
        """
        Export the scene.

        Parameters
        ----------
        scene : o3d.geometry.PointCloud
            The cloud to export.
        """
        points_path = os.path.join(self.points_folder, self.name_template.format(self.frame_number)+'.npy')
        labels_path = os.path.join(self.labels_folder, self.name_template.format(self.frame_number)+'.txt')
        # Export the point cloud
        points = np.asarray(scene.points)
        intensities = np.asarray(self.sensor.information)
        # Stack the points and intensities (N, 3) -> (N, 4)
        points = np.hstack((points, intensities.reshape(-1,1)))
        np.save(points_path, points)
        # Export the labels
        with open(labels_path, 'w') as f:
            for bbox, label in self.visible_bboxes:
                label = self.get_label(bbox, label)
                f.write(label) # write the label to the file as a new line
        self.frame_number += 1

    def get_label(self, bbox: o3d.geometry.OrientedBoundingBox, label: CarlaObject)->str:
        """
        Get the label of a bounding box.

        Parameters
        ----------
        bbox : o3d.geometry.OrientedBoundingBox
            The bounding box to get the label from.
        label : CarlaObject
            The object that the bounding box belongs to.

        Returns
        -------
        str
            The label of the bounding box.
        """
        center = bbox.center
        extent = bbox.extent
        yaw = rotation2euler(bbox.R)[2] # get the yaw angle from the rotation matrix
        name = label.name
        return f"{center[0]} {center[1]} {center[2]} {extent[0]} {extent[1]} {extent[2]} {yaw} {name}\n"
        

        

        