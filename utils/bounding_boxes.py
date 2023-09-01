
import numpy as np
import carla
import math
from typing import List
from utils.helper import deg2rad


def get_bounding_boxes(world: carla.World, object_list: List[carla.CityObjectLabel]) -> carla.BoundingBox:
    """
    Get the bounding boxes of the vehicles in the world

    Parameters
    ----------
    world : carla.World
        The world to get the bounding boxes from
    object_list : List[carla.CityObjectLabel]
        The list of objects to get the bounding boxes from

    Returns
    -------
    carla.BoundingBox
        The bounding boxes of the vehicles in the world
    """
    if not isinstance(object_list, list):
        object_list = [object_list]
    bounding_boxes: List = world.get_level_bbs(object_list[0].carla_type)  # Get the bounding boxes of the first vehicle, to initialize the list
    for vehicle in object_list:
        bounding_boxes.extend(world.get_level_bbs(vehicle))
    return bounding_boxes


def build_projection_matrix(w, h, fov) -> np.ndarray:
    """
    Build the projection matrix for a given width, height, and field of view.

    Parameters
    ----------
    w : int
        Width of the image
    h : int
        Height of the image
    fov : float
        Field of view angle in degrees

    Returns
    -------
    np.ndarray
        The projection matrix
    """
    focal = w / (2.0 * np.tan(deg2rad(fov) / 2.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K


def get_image_point(loc, K, w2c) -> np.ndarray:
    """
    Get the image coordinates of a 3D location using camera parameters.

    Parameters
    ----------
    loc : carla.Location
        The location of the object
    K : np.ndarray
        The camera matrix
    w2c : np.ndarray
        The world to camera transformation matrix

    Returns
    -------
    np.ndarray
        The image coordinates (x, y) of the 3D location
    """
    # Calculate 2D projection of 3D coordinate

    # Format the input coordinate (loc is a carla.Position object)
    point = np.array([loc.x, loc.y, loc.z, 1])
    # transform to camera coordinates
    point_camera = np.dot(w2c, point)

    # Now we must change from UE4's coordinate system to a "standard"
    # (x, y, z) -> (y, -z, x)
    # and we remove the fourth component as well
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    # Now project 3D->2D using the camera matrix
    point_img = np.dot(K, point_camera)
    # Normalize
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return point_img[0:2]


def close_bounding_boxes(bounding_boxes: carla.BoundingBox, vehicle_list: List[carla.CityObjectLabel], distance: float = 100) -> carla.BoundingBox:
    """
    Filter the bounding boxes based on the distance from the ego vehicle.

    Parameters
    ----------
    bounding_boxes : carla.BoundingBox
        The bounding boxes to filter
    vehicle_list : List[carla.CityObjectLabel]
        The list of vehicles to get the bounding boxes from
    distance : float, optional
        The distance to the ego vehicle, by default 100

    Returns
    -------
    carla.BoundingBox
        The filtered bounding boxes
    """
    if not isinstance(vehicle_list, list):
        vehicle_list = [vehicle_list]
    ego_vehicle = vehicle_list.pop(0)
    ego_vehicle_location = ego_vehicle.get_location()
    filtered_bounding_boxes = []
    for bounding_box in bounding_boxes:
        bounding_box_location = bounding_box.location
        if ego_vehicle_location.distance(bounding_box_location) < distance:
            filtered_bounding_boxes.append(bounding_box)
    return filtered_bounding_boxes


def world2sensor(array: np.ndarray, w2s: np.ndarray) -> np.ndarray:
    """
    Transform the coordinates from the world coordinate system to the sensor coordinate system.

    Parameters
    ----------
    array : np.ndarray
        The array to transform
    w2s : np.ndarray
        The transformation matrix

    Returns
    -------
    np.ndarray
        The transformed array
    """
    if array.shape[-1] == 3:
        array = np.hstack([array, np.ones((array.shape[0], 1))])
    array = np.dot(w2s, array.T).T
    array[:, 0] /= array[:, 3]
    array[:, 1] /= array[:, 3]
    array[:, 2] /= array[:, 3]
    array = array[:, :3]
    return array

def sensor2world(array: np.ndarray, s2w: np.ndarray) -> np.ndarray:
    """
    Transform the coordinates from the sensor coordinate system to the world coordinate system.

    Parameters
    ----------
    array : np.ndarray
        The array to transform
    s2w : np.ndarray
        The transformation matrix

    Returns
    -------
    np.ndarray
        The transformed array
    """
    if array.shape[-1] == 3:
        array = np.hstack([array, np.ones((array.shape[0], 1))])
    array = np.dot(s2w, array.T).T
    array[:, 0] /= array[:, 3]
    array[:, 1] /= array[:, 3]
    array[:, 2] /= array[:, 3]
    array = array[:, :3]
    return array

# def sensor2sensor
