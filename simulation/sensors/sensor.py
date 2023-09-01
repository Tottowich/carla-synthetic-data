
from typing import Dict, List, Tuple, Union, Optional
import carla
import numpy as np
import open3d as o3d
from queue import Queue, Empty
# from utils.helper import ThreadSafeStack as Queue
from copy import deepcopy
from utils.constants import SensorInformation, ColorConstants
from utils.helper import CustomTimer, to_list, carla_vec2np_array
DEBUG = True

class WrapperQueue(Queue):
    # Empty the queue before putting a new item
    def put(self, item, block:bool=True, timeout:float=None):
        while not self.empty():
            self.get()
        super().put(item, block=block, timeout=timeout)


def timing_decorator(func):
    name = func.__name__
    def timing_wrapper(self, image):
        """
        A timing decorator that measures the execution time of a function and updates the processing time statistics.

        Parameters
        ----------
        self : SensorCallback
            The SensorCallback object.
        image : np.ndarray
            The image data.

        Returns
        -------
        object
            The result of the callback function.
        """
        t_start = self.timer.time()
        result = func(self, image)
        t_end = self.timer.time()
        t = t_end - t_start
        
        if self.verbose:
            print(f"{self.name:<12} : {name} took {t:4.3e} seconds.")
        return result
    return timing_wrapper


class SensorCallback:
    """
    Base class for sensor callbacks.
    All sensor callbacks should inherit from this class.

    Parameters
    ----------
    name : str, optional
        The name of the callback.
    enabled : bool, optional
        Whether the callback is enabled or not.
    """

    def __init__(self, name: str = None, enabled: bool = True):
        """
        Initializes a SensorCallback object.

        Parameters
        ----------
        name : str, optional
            The name of the callback.
        enabled : bool, optional
            Whether the callback is enabled or not.
        """
        self._name = name if name is not None else self.__class__.__name__
        self.enabled = enabled
        self.sensor:Sensor = None

    def __call__(self, image: np.ndarray) -> np.ndarray:
        """
        Executes the callback function with the image as an argument.

        Parameters
        ----------
        image : np.ndarray
            The image data.

        Returns
        -------
        np.ndarray
            The result of the callback function.
        """
        return self.callback(image)

    def add_sensor(self, sensor: 'Sensor'):
        """
        Adds a sensor to the callback.

        Parameters
        ----------
        sensor : Sensor
            The sensor object.

        Raises
        ------
        AssertionError
            If the sensor is not of type 'Sensor'.
        AssertionError
            If the SensorCallback has already been added to a sensor.
        """
        assert isinstance(sensor, Sensor), f"Expected sensor to be of type 'Sensor', but got {type(sensor)}"
        assert self.sensor is None, "SensorCallback has already been added to a sensor"
        self.sensor = sensor
        self._add_sensor(sensor)

    def _add_sensor(self, sensor: 'Sensor'):
        """
        Placeholder method for adding a sensor to the callback.
        This method should be overridden in the subclass.

        Parameters
        ----------
        sensor : Sensor
            The sensor object.

        Raises
        ------
        NotImplementedError
            If the method is not implemented in the subclass.
        """
        raise NotImplementedError("Subclasses must implement the '_add_sensor' method.")

    def remove_sensor(self):
        """
        Removes the sensor from the callback.
        """
        self.sensor = None
        self._remove_sensor()

    def _remove_sensor(self):
        """
        Placeholder method for removing the sensor from the callback.
        This method should be overridden in the subclass.
        """
        pass

    def callback(self, image):
        """
        Placeholder method for the callback function.
        This method should be overridden in the subclass.

        Parameters
        ----------
        image : object
            The image data.

        Raises
        ------
        NotImplementedError
            If the method is not implemented in the subclass.
        """
        raise NotImplementedError("Subclasses must implement the 'callback' method.")
    @property
    def name(self) -> str:
        """
        Get the name of the callback.

        Returns
        -------
        str
            The name of the callback.
        """
        return self._name

    
class Sensor:
    """
    Base class for all sensors.
    All sensors should inherit from this class.

    Parameters
    ----------
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

    Notes
    -----
    The available sensor options can be found in the SensorInformation class found in utils/constants.py. The options are the attributes of the SensorOptions class.
    """

    is_2d = None  # Whether the sensor is 2D or 3D

    def __init__(
        self,
        world: carla.World,
        transform: carla.Transform,
        attached: carla.Actor,
        sensor_options: Dict[str, str] = None,
        verbose: bool = False,
        display_manager: "DisplayManager" = None,
        display_pos: Tuple[int, int] = None,
        callbacks: List[SensorCallback] = None,
        render: bool = False,
        *args,
        **kwargs,
    ):
        """
        Initializes a Sensor object.

        Parameters
        ----------
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
        render : bool, optional
            Whether to render the sensor data. Defaults to False. Rendering implies that the sensor data is post-processed.
        """
        self.verbose = verbose
        self.surface = None
        self.world = world
        self._frame_id = 0
        self.timer = CustomTimer()
        self.processed = None
        self.scene = None if not hasattr(self, "scene") else self.scene
        self.queue = WrapperQueue(maxsize=1)
        self.render = render if display_manager is None else True
        self.attached = attached  # The actor to which the sensor is attached
        self.callbacks = to_list(callbacks, SensorCallback)
        self.display_manager = display_manager
        self.sensor_options = sensor_options

        if self.display_manager:
            self.display_size = display_manager.get_display_size()
            display_manager.add_sensor(self, display_pos)
        if sensor_options is not None:
            sensor_options = {k: str(v) for k, v in sensor_options.items() if v is not None}
        else:
            sensor_options = {}
        self.sensor = self.init_sensor(transform, attached, sensor_options)
        if self.verbose:
            print(f"Sensor {self.name} initialized with options {self.sensor_options}")

        # if self.render:
        #     self.sensor.listen(lambda data : self.queue.put(self.render_data(data)))
        # else:
        #     self.sensor.listen(lambda data : self.queue.put(self.process_data(data)))
        self.sensor.listen(lambda data : self.queue.put(data))

        for callback in self.callbacks:
            callback.add_sensor(self)

    def init_sensor(self, transform, attached, sensor_options) -> carla.Sensor:
        """
        Initializes the sensor object.

        Parameters
        ----------
        transform : carla.Transform
            The transform of the sensor used to spawn it.
        attached : carla.Actor
            The actor to which the sensor is attached.
        sensor_options : Dict[str, str]
            A dictionary containing the sensor options.

        Returns
        -------
        carla.Sensor
            The initialized sensor object.
        """
        raise NotImplementedError("Subclasses must implement the 'init_sensor' method.")

    def process_data(self, data: carla.SensorData) -> np.ndarray:
        """
        Processes the sensor data.

        Parameters
        ----------
        data : carla.SensorData
            The sensor data.

        Returns
        -------
        np.ndarray
            The processed data.
        """
        assert isinstance(data, carla.SensorData), f"Expected data to be of type 'carla.SensorData', but got {type(data)}"
        self._update_frame_id(data)
        if not self.is_synched or not self.sensor.is_alive or not self.sensor.is_listening:
            return None
        self.processed = self._process_data(data)
        # self._empty_queue()
        return self.processed

    @timing_decorator
    def render_data(self, data: carla.SensorData):
        """
        Renders the sensor data and applies callbacks.

        Parameters
        ----------
        data : carla.SensorData
            The sensor data.

        Returns
        -------
        np.ndarray
            The rendered scene.
        
        Raises
        ------
        AssertionError
            If the given data is not of type 'carla.SensorData'.
        AssertionError
            If the processed data is not of type 'np.ndarray'.
        """
        processed = self.process_data(data)
        if processed is None:
            return None
        assert isinstance(self.processed, np.ndarray), f"Expected processed data to be of type 'np.ndarray', but got {type(self.processed)}"
        scene = self._render_data(self.processed.copy())
        if not self.sensor.is_alive:    
            return None
        for callback in self.callbacks:
            scene = callback(scene)
        self.scene = scene
        # self._empty_queue()
        return self.scene
    
    def _render_data(self, array: np.ndarray) -> np.ndarray:
        """
        Renders the sensor data.

        Parameters
        ----------
        array : np.ndarray
            The processed sensor data.

        Returns
        -------
        np.ndarray
            The rendered scene.
        """
        raise NotImplementedError("Subclasses must implement the '_render_data' method.")

    def _process_data(self, data: carla.SensorData) -> np.ndarray:
        """
        Processes the sensor data.

        Parameters
        ----------
        data : carla.SensorData
            The sensor data.

        Returns
        -------
        np.ndarray
            The processed data.
        """
        raise NotImplementedError("Subclasses must implement the '_process_data' method.")

    def _update_frame_id(self, data: carla.SensorData):
        """
        Updates the frame ID.

        Parameters
        ----------
        data : carla.SensorData
            The sensor data.
        """
        self._frame_id = data.frame
    
    def transform(self, matrix:bool=False, numpy:bool=False) -> Union[carla.Transform, np.ndarray]:
        """
        Get the transform of the sensor.

        Parameters
        ----------
        matrix : bool, optional
            Whether to return the transform as a matrix. Defaults to False.
        numpy : bool, optional
            Whether to return the transform as a NumPy array. Defaults to False.

        Returns
        -------
        carla.Transform or np.ndarray
            The transform of the sensor.
        """
        T = self.sensor.get_transform()
        if matrix:
            T = T.get_matrix()
        if numpy:
            if not matrix:
                T = T.get_matrix()
            T = np.array(T)
        return T        
    def inverse_transform(self, numpy:bool=False) -> Union[carla.Transform, np.ndarray]:
        """"
        Get the inverse transform of the sensor.

        Parameters
        ----------
        numpy : bool, optional
            Whether to return the transform as a NumPy array. Defaults to False.

        Returns
        -------
        carla.Transform or np.ndarray
            The inverse transform of the sensor.
        """
        I_T = self.sensor.get_transform().get_inverse_matrix()
        if numpy:
            return np.array(I_T)
        return I_T
            
    def location(self, numpy: bool = False) -> Union[carla.Location, np.ndarray]:
        """
        Get the location of the sensor.

        Parameters
        ----------
        numpy : bool, optional
            Whether to return the location as a NumPy array. Defaults to False.

        Returns
        -------
        carla.Location or np.ndarray
            The location of the sensor.
        """
        loc = self.transform().location
        if numpy:
            return carla_vec2np_array(loc)
        return loc

    def forward_vector(self, numpy: bool = False, normalized: bool = True) -> Union[carla.Vector3D, np.ndarray]:
        """
        Get the forward vector of the sensor.

        Parameters
        ----------
        numpy : bool, optional
            Whether to return the vector as a NumPy array. Defaults to False.
        normalized : bool, optional
            Whether to return the normalized vector. Defaults to True.

        Returns
        -------
        carla.Vector3D or np.ndarray
            The forward vector of the sensor.
        """
        vec = self.transform().get_forward_vector()
        if normalized:
            vec = vec / vec.length()
        if numpy:
            return carla_vec2np_array(vec)
        return vec

    def destroy(self):
        """
        Destroy the sensor.
        """
        self.sensor.destroy()
    
    def get_scene(self, block:bool=True, timeout:float = 5e-3) -> Union[np.ndarray, o3d.geometry.PointCloud]:
        """
        Get the scene.

        Parameters
        ----------
        block : bool, optional
            Whether to block the call until the scene is available. Defaults to True.
        timeout : float, optional
            The timeout in seconds. Defaults to 1e-3 (1 ms).
        
        Returns
        -------
        np.ndarray or o3d.geometry.PointCloud
            The scene: either a NumPy array or an Open3D point cloud.
        """
        try:
            # return self.queue.get(block=block, timeout=timeout)
            if self.render:
                return self.render_data(self.queue.get(block=block, timeout=timeout))
            else:
                return self.process_data(self.queue.get(block=block, timeout=timeout))
        except Exception as e:
            if self.verbose and e is not None and not isinstance(e, Empty):
                print(f"Could not get scene from sensor {self.name}. Error: {e}")
                if DEBUG:
                    raise e
            return None
    def _empty_queue(self):
        """
        Empty the queue.
        """
        while not self.queue.empty():
            self.queue.get()
    def world2sensor(self, transform:carla.Transform)->carla.Transform:
        """
        Transform the given transform from the world coordinate system to the sensor coordinate system.

        Parameters
        ----------
        transform : carla.Transform
            The transform to transform.

        Returns
        -------
        carla.Transform
            The transformed transform.

        """
        return transform * self.transform().get_inverse_matrix()
    def w2s(self,numpy:bool=False)->Union[carla.Transform, np.ndarray]:
        """
        Get the world-to-sensor transformation matrix.

        Returns
        -------
        np.ndarray
            The world-to-sensor transformation matrix.
        """
        w2s = self.transform().get_inverse_matrix()
        if numpy:
            return np.array(w2s)
        return w2s
    def s2w(self,numpy:bool=False)->Union[carla.Transform, np.ndarray]:
        """
        Get the sensor-to-world transformation matrix.

        Returns
        -------
        np.ndarray
            The sensor-to-world transformation matrix.
        """
        s2w = self.transform().get_matrix()
        if numpy:
            return np.array(s2w)
        return s2w
    
    @property
    def is_synched(self) -> bool:
        """
        Get whether the sensor is in sync with the world.

        Returns
        -------
        bool
            Whether the sensor is in sync with the world.
        """
        world_frame = self.world.get_snapshot().frame
        synched = self.frame_id == world_frame
        if not synched and self.verbose:
            print(f"{self.name} - Sensor is not synched with the world. Skipping frame {self.frame_id}, world frame is {world_frame}")
        return synched
    
    @property
    def name(self) -> str:
        """
        Get the name of the sensor.

        Returns
        -------
        str
            The name of the sensor.
        """
        return self.__class__.__name__
    @property
    def available_options(self) -> List[str]:
        """
        Get the available sensor options.

        Returns
        -------
        List[str]
            The available sensor options.
        """
        return list(SensorInformation.get_sensor_options(self.name).keys())
    @property
    def default_options(self) -> Dict[str, str]:
        """
        Get the default sensor options.

        Returns
        -------
        Dict[str, str]
            The default sensor options.
        """
        return SensorInformation.get_sensor_options(self.name)
    @property
    def sensor_range(self) -> float:
        """
        Get the sensor range.

        Returns
        -------
        float
            The sensor range.
        
        Notes
        -----

        The sensor range is the maximum distance at which the sensor can detect objects.
        Default value is infinite before it is set by the default blueprint.
        """
        return np.inf
    @property
    def frame_id(self) -> int:
        """
        Get the current frame ID.

        Returns
        -------
        int
            The current frame ID.
        """
        return self._frame_id
   