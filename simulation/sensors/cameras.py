from typing import Dict, List, Optional, Tuple, Union

import carla
import numpy as np
import open3d as o3d

from simulation.sensors.sensor import Sensor, SensorCallback
from utils.bounding_boxes import build_projection_matrix
from utils.constants import SensorInformation


class Camera(Sensor):
    """
    Base class for all Cameras.

    Parameters
    ----------
    dformat : carla.ColorConverter
        The data format of the camera.
    blueprint : str
        The blueprint of the camera.
    *args
        Variable length argument list.
    **kwargs
        Additional keyword arguments passed to the Sensor class.

    Keyword Args
    ------------
    world : carla.World
        The world object.
    transform : carla.Transform
        The transform of the sensor used to spawn it.
    attached : carla.Actor
        The actor to which the sensor is attached.
    sensor_options : Dict[str, str]
        A dictionary containing the sensor options.
    verbose : bool, optional
        Whether to enable verbose mode. Defaults to False.
    display_manager : DisplayManager, optional
        The display manager for rendering. Defaults to None.
    display_pos : Tuple[int, int], optional
        The position of the sensor on the display. Defaults to None.
    callbacks : List[SensorCallback], optional
        List of sensor callbacks. Defaults to None.
    """

    is_2d = True

    def __init__(self, dformat: carla.ColorConverter, blueprint: str, *args, **kwargs):
        """
        Initializes a Camera object.

        Parameters
        ----------
        dformat : carla.ColorConverter
            The data format of the camera.
        blueprint : str
            The blueprint of the camera.
        *args
            Variable length argument list.
        **kwargs
            Additional keyword arguments passed to the Sensor class.
        """
        assert isinstance(dformat, carla.ColorConverter), f"Expected dformat to be of type 'carla.ColorConverter', but got {type(dformat)}"
        self.dformat = dformat
        self.blueprint = blueprint
        self._H = None
        self._W = None
        self._K = None
        self._fov = None
        super().__init__(*args, **kwargs)

    def init_sensor(self, transform, attached, sensor_options):
        """
        Initializes the camera sensor.

        Parameters
        ----------
        transform : carla.Transform
            The transform of the camera.
        attached : carla.Actor
            The actor to which the camera is attached.
        sensor_options : Dict[str, str]
            A dictionary containing the sensor options.

        Returns
        -------
        carla.Actor
            The camera actor.
        """
        camera_bp = self.world.get_blueprint_library().find(self.blueprint)
        if self.render:
            camera_bp.set_attribute('image_size_x', str(self.display_size[0]))
            camera_bp.set_attribute('image_size_y', str(self.display_size[1]))
            # Remove 'image_size_x' and 'image_size_y' from sensor_options
            sensor_options.pop('image_size_x', None)
            sensor_options.pop('image_size_y', None)
    
        for key in sensor_options:
            camera_bp.set_attribute(key, sensor_options[key])

        camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
        # Get fov:
        self._fov = camera_bp.get_attribute('fov').as_float()
        # Get H and W:
        self._W = camera_bp.get_attribute('image_size_x').as_int()
        self._H = camera_bp.get_attribute('image_size_y').as_int()
        # Get K matrix:
        self._K = build_projection_matrix(self.W, self.H, self.fov)
        return camera

    def _process_data(self, data: SensorInformation.RGBCamera.DATATYPE) -> np.ndarray:
        """
        Converts the raw data from the camera sensor to a numpy array.

        Parameters
        ----------
        data : SensorInformation.RGBCamera.DATATYPE
            The raw camera data.

        Returns
        -------
        np.ndarray
            The processed camera data as a numpy array.
        """
        if isinstance(self, DepthCamera):
            assert self.dformat == carla.ColorConverter.Depth or self.dformat == carla.ColorConverter.LogarithmicDepth, f"Expected dformat to be 'carla.ColorConverter.Depth' or 'carla.ColorConverter.LogarithmicDepth', but got {self.dformat}"
        if isinstance(self, SemanticSegmentationCamera):
            assert self.dformat == carla.ColorConverter.CityScapesPalette, f"Expected dformat to be 'carla.ColorConverter.CityScapesPalette', but got {self.dformat}"
        if isinstance(self, InstanceSegmentationCamera):
            assert self.dformat == carla.ColorConverter.CityScapesPalette, f"Expected dformat to be 'carla.ColorConverter.CityScapesPalette', but got {self.dformat}"
        if isinstance(self, RGBCamera):
            assert self.dformat == carla.ColorConverter.Raw, f"Expected dformat to be 'carla.ColorConverter.Raw', but got {self.dformat}"
        data.convert(self.dformat)
        array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (self.H, self.W, 4))
        array = array[:, :, :3]
        return array
    
    def _render_data(self, array: np.ndarray) -> np.ndarray:
        """
        Converts the BGR image generated by Camera._process_data to RGB for rendering.

        Parameters
        ----------
        array : np.ndarray
            The BGR image generated by Camera._process_data.

        Returns
        -------
        np.ndarray
            The RGB image for rendering.
        """
        array = array[:, :, ::-1]
        return np.ascontiguousarray(array)  # RGB image in np.uint8, shape=(height, width, 3)

    @property
    def H(self) -> int:
        """
        Get the height of the camera image.

        Returns
        -------
        int
            The height of the camera image.
        """
        return self._H

    @property
    def W(self) -> int:
        """
        Get the width of the camera image.

        Returns
        -------
        int
            The width of the camera image.
        """
        return self._W

    @property
    def K(self) -> np.ndarray:
        """
        Get the intrinsic calibration matrix (K matrix) of the camera.

        Returns
        -------
        np.ndarray
            The intrinsic calibration matrix.
        """
        return self._K

    @property
    def fov(self) -> float:
        """
        Get the field of view (FOV) of the camera.

        Returns
        -------
        float
            The field of view of the camera.
        """
        return self._fov


class RGBCamera(Camera):
    """
    RGB Camera class.
    """
    def __init__(self, *args, **kwargs):
        """
        Initializes an RGBCamera object.

        Parameters
        ----------
        *args
            Variable length argument list.
        **kwargs
            Additional keyword arguments passed to the Camera class.

        Keyword Args
        ------------
        world : carla.World
            The world object.
        transform : carla.Transform
            The transform of the sensor used to spawn it.
        attached : carla.Actor
            The actor to which the sensor is attached.
        sensor_options : Dict[str, str]
            A dictionary containing the sensor options.
        verbose : bool, optional
            Whether to enable verbose mode. Defaults to False.
        display_manager : DisplayManager, optional
            The display manager for rendering. Defaults to None.
        display_pos : Tuple[int, int], optional
            The position of the sensor on the display. Defaults to None.
        callbacks : List[SensorCallback], optional
            List of sensor callbacks. Defaults to None.
        """
        super().__init__(SensorInformation.RGBCamera.DATATYPE,
                         SensorInformation.RGBCamera.blueprint, *args, **kwargs)


class SemanticSegmentationCamera(Camera):
    """
    Semantic Segmentation Camera class.
    """
    def __init__(self, *args, **kwargs):
        """
        Initializes a Semantic Segmentation Camera object.

        Parameters
        ----------
        *args
            Variable length argument list.
        **kwargs
            Additional keyword arguments passed to the Camera class.

        Keyword Args
        ------------
        world : carla.World
            The world object.
        transform : carla.Transform
            The transform of the sensor used to spawn it.
        attached : carla.Actor
            The actor to which the sensor is attached.
        sensor_options : Dict[str, str]
            A dictionary containing the sensor options.
        verbose : bool, optional
            Whether to enable verbose mode. Defaults to False.
        display_manager : DisplayManager, optional
            The display manager for rendering. Defaults to None.
        display_pos : Tuple[int, int], optional
            The position of the sensor on the display. Defaults to None.
        callbacks : List[SensorCallback], optional
            List of sensor callbacks. Defaults to None.
        """
        super().__init__(SensorInformation.SemanticSegmentationCamera.DATATYPE,
                         SensorInformation.SemanticSegmentationCamera.blueprint, *args, **kwargs)

class InstanceSegmentationCamera(Camera):
    """
        Initializes an Instance Segmentation Camera object.

    """
    def __init__(self,  *args, **kwargs):
        """
        Initializes a Semantic Segmentation Camera object.

        Keword Args:
            world (carla.World): The world object.
            transform (carla.Transform): The transform of the sensor used to spawn it.
            attached (carla.Actor): The actor to which the sensor is attached.
            sensor_options (Dict[str, str]): A dictionary containing the sensor options.
            verbose (bool, optional): Whether to enable verbose mode. Defaults to False.
            display_manager (DisplayManager, optional): The display manager for rendering. Defaults to None.
            display_pos (Tuple[int, int], optional): The position of the sensor on the display. Defaults to None.
            callbacks (List[SensorCallback], optional): List of sensor callbacks. Defaults to None.
        """
        super().__init__(SensorInformation.InstanceSegmentationCamera.DATATYPE,
                        SensorInformation.InstanceSegmentationCamera.blueprint,  *args, **kwargs)

class DepthCamera(Camera):
    """
    Depth Camera class.
    """
    def __init__(self, logarithmic: bool = True, *args, **kwargs):
        """
        Initializes a Depth Camera object.

        Parameters
        ----------
        logarithmic : bool, optional
            Whether to use logarithmic depth or not.
        *args
            Variable length argument list.
        **kwargs
            Additional keyword arguments passed to the Camera class.

        Keyword Args
        ------------
        world : carla.World
            The world object.
        transform : carla.Transform
            The transform of the sensor used to spawn it.
        attached : carla.Actor
            The actor to which the sensor is attached.
        sensor_options : Dict[str, str]
            A dictionary containing the sensor options.
        verbose : bool, optional
            Whether to enable verbose mode. Defaults to False.
        display_manager : DisplayManager, optional
            The display manager for rendering. Defaults to None.
        display_pos : Tuple[int, int], optional
            The position of the sensor on the display. Defaults to None.
        callbacks : List[SensorCallback], optional
            List of sensor callbacks. Defaults to None.
        """
        assert isinstance(logarithmic, bool), f"DepthCamera 'logarithmic' must be 'bool' not {type(logarithmic)}"
        self.logarithmic = logarithmic
        dformat = (
            SensorInformation.DepthCamera.DATATYPE
            if not logarithmic
            else SensorInformation.LogarithmicDepthCamera.DATATYPE
        )
        super().__init__(dformat, SensorInformation.DepthCamera.blueprint, *args, **kwargs)

    def normalize(self, data: np.ndarray) -> np.ndarray:
        """
        Normalizes the depth camera data.

        Reference code from: https://carla.readthedocs.io/en/latest/ref_sensors/#depth-camera

        Parameters
        ----------
        data : np.ndarray
            The depth camera data.

        Returns
        -------
        np.ndarray
            The normalized depth camera data.
        """
        assert isinstance(data, np.ndarray), f"Expected data to be of type 'np.ndarray', but got {type(data)}. Make sure you're using the processed data from the DepthCamera."
        R, G, B = data[:, :, 0], data[:, :, 1], data[:, :, 2]
        normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
        return normalized

    def in_meters(self, data: np.ndarray) -> np.ndarray:
        """
        Converts the normalized depth camera data to meters.

        Parameters
        ----------
        data : np.ndarray
            The normalized depth camera data.

        Returns
        -------
        np.ndarray
            The depth camera data in meters.
        """
        return 1000 * self.normalize(data)
