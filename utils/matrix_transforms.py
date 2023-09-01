import numpy as np
import carla
from simulation.sensors.sensor import Sensor
from transforms3d.euler import euler2mat, mat2euler


def transform2target(array: np.ndarray, tranform_matrix: np.ndarray) -> np.ndarray:
    """
    Transform the coordinates one coordinate system to the sensor coordinate system using a transformation matrix.

    Parameters
    ----------
    array : np.ndarray
        The array to transform
    tranform_matrix : np.ndarray
        The transformation matrix

    Returns
    -------
    np.ndarray
        The transformed array
    """
    array = check_shape(array)
    array = np.dot(tranform_matrix, array.T).T
    array[:, 0] /= array[:, 3]
    array[:, 1] /= array[:, 3]
    array[:, 2] /= array[:, 3]
    array = array[:, :3]
    return array

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
    
    return transform2target(array, w2s)


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
    return transform2target(array, s2w)

def check_shape(array:np.ndarray):
    """
    Check if the array has the correct shape for the transformation.

    Parameters
    ----------
    array : np.ndarray
        The array to check
    
    Returns
    -------
    np.ndarray
        The array with the correct shape
    """
    if array.shape[-1] == 3:
        array = np.hstack([array, np.ones((array.shape[0], 1))])
    return array

def relative_transform_matrix(source:Sensor,target:Sensor):
    """
    Compute the transformation matrix from source to target.

    Parameters
    ----------
    source : Sensor
        The sensor to transform from
    target : Sensor
        The sensor to transform to

    Returns
    -------
    np.ndarray
        The transformation matrix
    """
    TaI = source.inverse_transform(numpy=True)
    Tb = target.transform(numpy=True)
    Tab = np.dot(Tb,TaI)
    return Tab

def sensor2sensor_transform(source:Sensor,target:Sensor,array:np.ndarray):
    """
    Transform the coordinates from source to target

    Parameters
    ----------
    source : Sensor
        The sensor to transform from
    target : Sensor
        The sensor to transform to
    array : np.ndarray
        The array to transform

    Returns
    -------
    np.ndarray
        The transformed array
    """
    Tab = relative_transform_matrix(source, target)
    return transform2target(array, Tab)

def position_matrix(xyz:np.ndarray) -> np.ndarray:
    """
    Create a position matrix from a position vector.

    Parameters
    ----------
    xyz : np.ndarray
        The position vector

    Returns
    -------
    np.ndarray
        The position matrix
    """

    eye = np.eye(4)
    eye[:3,3] = xyz
    return eye
def rotation2euler(rotation:np.ndarray)->np.ndarray:
    """
    Convert a rotation matrix to euler angles.

    Parameters
    ----------
    rotation : np.ndarray
        The rotation matrix

    Returns
    -------
    np.ndarray
        The euler angles
    """
    return mat2euler(rotation)