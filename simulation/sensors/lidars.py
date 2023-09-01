from copy import deepcopy
from typing import Dict, List, Optional, Tuple, Union

import carla
import numpy as np
import open3d as o3d

from simulation.sensors.sensor import Sensor, SensorCallback
from simulation.sensors.imu import IMU
from utils.constants import ColorConstants, SensorInformation
from utils.matrix_transforms import  transform2target, relative_transform_matrix, sensor2world, world2sensor
def calculate_point_spacing(points_per_second: int, channels: int, rotation_frequency: int, horizontal_fov: float, vertical_fov: float) -> float:
    """
    Calculates the point spacing of a LiDAR sensor in degrees.

    Parameters
    ----------
    points_per_second : int
        The number of points per second.
    channels : int
        The number of channels.
    rotation_frequency : int
        The rotation frequency.
    horizontal_fov : float
        The horizontal field of view.
    vertical_fov : float
        The vertical field of view.

    Returns
    -------
    tuple
        The point spacing in degrees. (horizontal, vertical)
    """
    vert_spacing = vertical_fov / channels
    horiz_spacing = horizontal_fov*rotation_frequency / points_per_second
    return horiz_spacing, vert_spacing

class LiDAR2Open3D_Callback(SensorCallback):
    """
    Update the Open3D point cloud with new data from Numpy array containing LiDAR points and their information.

    Parameters
    ----------
    name : str, optional
        The name of the callback. Defaults to 'LiDAR2Open3D_Callback'.
    enabled : bool, optional
        Whether the callback is enabled or not.
    """
    def _add_sensor(self, sensor: 'LiDAR_Sensor'):
        """
        Add a sensor to the LiDAR2Open3D_Callback.

        Parameters
        ----------
        sensor : LiDAR_Sensor
            The camera sensor to be added.

        Raises
        ------
        AssertionError
            If sensor is not of type 'LiDAR_Sensor'.
        AssertionError
            If sensor does not have a 'scene' attribute.
        AssertionError
            If sensor.scene is not of type 'o3d.geometry.PointCloud'.
        """
        assert isinstance(sensor, LiDAR_Sensor), f"Expected sensor to be of type 'LiDAR_Sensor', but got {type(sensor)}"
        assert hasattr(sensor, 'scene'), f"Expected sensor to have a 'scene' attribute, but got {sensor}"
        assert isinstance(sensor.scene, o3d.geometry.PointCloud), f"Expected sensor.scene to be of type 'o3d.geometry.PointCloud', but got {type(sensor.scene)}"
        self.scene = sensor.scene
    def callback(self, array: np.ndarray):
        # Negating y coordinate for Open3D reference frame.
        array[:, 1] = -array[:, 1]
        self.scene.points = o3d.utility.Vector3dVector(array[:, :3])
        self.scene.colors = o3d.utility.Vector3dVector(array[:, 3:])
        return self.scene

class LiDAR_Sensor(Sensor):
    """
    LiDAR Sensor class.

    Attributes
    ----------
    dims2setup : Dict[int, Dict[str, Any]]
        A dictionary containing the setup for different LiDAR sensors (Regular/Semantic).
    """

    dims2setup = {
        4: {
            "blueprint": SensorInformation.LiDAR.blueprint,
            "range": SensorInformation.LiDAR.OPTIONS.range,
            "dtype": 'f4'
        },
        6: {
            "blueprint": SensorInformation.SemanticLiDAR.blueprint,
            "range": SensorInformation.SemanticLiDAR.OPTIONS.range,
            "dtype": [('x', np.float32), ('y', np.float32), ('z', np.float32),
                      ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]
        }
    }

    def __init__(self, dims: int = 4, *args, **kwargs):
        """
        Initializes a LiDAR Sensor object.

        Parameters
        ----------
        dims : int, optional
            The number of dimensions of the LiDAR sensor data. Defaults to 4.
        *args
            Variable length argument list.
        **kwargs
            Arbitrary keyword arguments.

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
        
        Raises
        ------
        AssertionError
            If `dims` is not a positive integer.

        Attributes
        ----------
        dims : int
            The number of dimensions of the LiDAR sensor data.
        scene : Union[None, open3d.geometry.PointCloud]
            The point cloud data.
        blueprint : str
            The blueprint of the LiDAR sensor.
        range : float
            The range of the LiDAR sensor.
        dtype : Union[None, np.dtype]
            The data type of the LiDAR sensor data.
        information : Union[None, np.ndarray]
            The information of the LiDAR sensor data.
        point_spacing : Union[None, float]
            The point spacing of the LiDAR sensor data in degrees.

        """
        assert isinstance(dims, int) and dims > 0, "dims must be a positive integer"
        self.dims = dims
        self.scene = o3d.geometry.PointCloud() if not self.is_2d else None
        self.blueprint = self.dims2setup[dims]["blueprint"]
        self.range = self.dims2setup[dims]["range"]
        self.dtype = self.dims2setup[dims]["dtype"]
        self.information = None
        self.point_spacing = None
        # Prepend 'LiDAR_Callback' to the list of callbacks
        if not self.is_2d:
            if 'callbacks' not in kwargs:
                kwargs['callbacks'] = [LiDAR2Open3D_Callback(name='LiDAR_Callback')]
            else:
                kwargs['callbacks'].insert(0, LiDAR2Open3D_Callback())
        super().__init__(*args, **kwargs)

    def init_sensor(self, transform, attached, sensor_options):
        """
        Initializes the LiDAR sensor.

        Parameters
        ----------
        transform : carla.Transform
            The transform of the LiDAR sensor.
        attached : carla.Actor
            The actor to which the LiDAR sensor is attached.
        sensor_options : Dict[str, str]
            A dictionary containing the sensor options.

        Returns
        -------
        carla.Actor
            The LiDAR actor.
        """
        lidar_bp = self.world.get_blueprint_library().find(self.blueprint)
        lidar_bp.set_attribute('range', str(self.range))  # Set default range
        if self.dims == 4:
            lidar_bp.set_attribute('dropoff_general_rate',
                                lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit',
                                lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity',
                                lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

        for key in sensor_options:
            lidar_bp.set_attribute(key, sensor_options[key])
        if self.dims == 4:
            self.atmosphere_attenuation_rate = lidar_bp.get_attribute('atmosphere_attenuation_rate').as_float()

        self._pps = lidar_bp.get_attribute('points_per_second').as_int()
        self._channels = lidar_bp.get_attribute('channels').as_int()
        self._rotation_frequency = lidar_bp.get_attribute('rotation_frequency').as_float()
        self._fov = lidar_bp.get_attribute('horizontal_fov').as_float()
        self._upper_fov = lidar_bp.get_attribute('upper_fov').as_float()
        self._lower_fov = lidar_bp.get_attribute('lower_fov').as_float()
        self._sensor_range = lidar_bp.get_attribute('range').as_float()
        self._point_spacing = calculate_point_spacing(
            self.pps,
            self.channels,
            self.rotation_frequency,
            self.fov,
            self.upper_fov + self.lower_fov
        )


        lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
        return lidar

    def _process_data(self, data: carla.LidarMeasurement) -> np.ndarray:
        """
        Processes the raw LiDAR data.

        Parameters
        ----------
        data : carla.LidarMeasurement
            The raw LiDAR data.

        Returns
        -------
        np.ndarray
            The processed LiDAR data as a numpy array.
        """
        points = np.frombuffer(data.raw_data, dtype=np.dtype(self.dtype))
        points, self.information = self._split_data(points)
        return points
            

    def _get_color(self, array: np.ndarray):
        """
        Gets the color for rendering the LiDAR data.

        Parameters
        ----------
        array : np.ndarray
            The LiDAR data array.

        Raises
        ------
        NotImplementedError
            Subclasses must implement the '_get_color' method.
        """
        raise NotImplementedError("Subclasses must implement the '_get_color' method.")

    def _split_data(self, array: np.ndarray):
        """
        Splits the LiDAR data into points and parameters.

        Parameters
        ----------
        array : np.ndarray
            The LiDAR data array.

        Raises
        ------
        NotImplementedError
            Subclasses must implement the '_split_data' method.
        """
        raise NotImplementedError("Subclasses must implement the '_split_data' method.")

    def _render_data(weak_self, array: np.ndarray) -> np.ndarray:
        """
        Renders the LiDAR data. To be (x,y,z,r,g,b)

        Parameters
        ----------
        array : np.ndarray
            The LiDAR data array.

        Returns
        -------
        o3d.geometry.PointCloud
            The rendered LiDAR data as an Open3D point cloud.
        """
        color = weak_self._get_color(weak_self.information)
        # Transforming from Unreal to Open3D coordinates, negative y
        points = np.concatenate((array[:, :3], color), axis=1)
        return points

    def intensity2color(self, intensity: np.ndarray) -> np.ndarray:
        """
        Converts intensity values to colors.

        Parameters
        ----------
        intensity : np.ndarray
            The intensity values.

        Returns
        -------
        np.ndarray
            The colors corresponding to the intensity values.
        """
        intensity_color = 1.0 - np.log(intensity) / np.log(np.exp(-self.atmosphere_attenuation_rate * 100))
        int_color = np.c_[
            np.interp(intensity_color, ColorConstants.VID_RANGE, ColorConstants.VIRIDIS[:, 0]),
            np.interp(intensity_color, ColorConstants.VID_RANGE, ColorConstants.VIRIDIS[:, 1]),
            np.interp(intensity_color, ColorConstants.VID_RANGE, ColorConstants.VIRIDIS[:, 2])
        ]
        return int_color
    @property
    def pps(self) -> int:
        """
        Get the points per second of the LiDAR sensor.

        Returns
        -------
        int
            The points per second of the LiDAR sensor.
        """
        return self._pps
    @property
    def channels(self) -> int:
        """
        Get the number of channels of the LiDAR sensor.

        Returns
        -------
        int
            The number of channels of the LiDAR sensor.
        """
        return self._channels
    @property
    def rotation_frequency(self) -> float:
        """
        Get the rotation frequency of the LiDAR sensor.

        Returns
        -------
        float
            The rotation frequency of the LiDAR sensor.
        """
        return self._rotation_frequency
    @property
    def fov(self) -> float:
        """
        Get the horizontal field of view of the LiDAR sensor.

        Returns
        -------
        float
            The horizontal field of view of the LiDAR sensor.
        """
        return self._fov
    @property
    def upper_fov(self) -> float:
        """
        Get the upper field of view of the LiDAR sensor.

        Returns
        -------
        float
            The upper field of view of the LiDAR sensor.
        """
        return self._upper_fov
    @property
    def lower_fov(self) -> float:
        """
        Get the lower field of view of the LiDAR sensor.

        Returns
        -------
        float
            The lower field of view of the LiDAR sensor.
        """
        return self._lower_fov
    @property
    def sensor_range(self) -> float:
        """
        Get the range of the LiDAR sensor.

        Returns
        -------
        float
            The range of the LiDAR sensor.
        """
        return self._sensor_range




class LiDAR(LiDAR_Sensor):
    """
    LiDAR class representing a LiDAR sensor.

    Inherits from LiDAR_Sensor class.

    Attributes
    ----------
    is_2d : bool
        Flag indicating if the LiDAR sensor is 2D (False for LiDAR).
    """

    is_2d = False

    def __init__(self, *args, **kwargs):
        """
        Initializes a LiDAR object.

        Parameters
        ----------
        *args
            Variable length argument list.
        **kwargs
            Arbitrary keyword arguments.
        """
        super().__init__(dims=4, *args, **kwargs)

    def _get_color(self, array: np.ndarray):
        """
        Gets the color representation of LiDAR points.

        Parameters
        ----------
        array : np.ndarray
            Array containing LiDAR point information.

        Returns
        -------
        np.ndarray
            Color representation of LiDAR points.
        """
        return self.intensity2color(array[:, -1])

    def _split_data(self, array: np.ndarray):
        """
        Splits the LiDAR data array into points and additional information.

        Parameters
        ----------
        array : np.ndarray
            Array containing LiDAR point information.

        Returns
        -------
        Tuple[np.ndarray, np.ndarray]
            Tuple containing points and additional information.
        """
        array = np.reshape(array, (-1, self.dims))
        information = array[:, 3:]
        return array, information

class SemanticLiDAR(LiDAR_Sensor):
    """
    SemanticLiDAR class representing a semantic LiDAR sensor.

    Inherits from LiDAR_Sensor class.

    Attributes
    ----------
    is_2d : bool
        Flag indicating if the LiDAR sensor is 2D (False for LiDAR).
    """

    is_2d = False

    def __init__(self, *args, **kwargs):
        """
        Initializes a SemanticLiDAR object.

        Parameters
        ----------
        *args
            Variable length argument list.
        **kwargs
            Arbitrary keyword arguments.

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
        super().__init__(dims=6, *args, **kwargs)

    def _split_data(self, array: np.ndarray):
        """
        Splits the SemanticLiDAR data array into points and additional information.

        Parameters
        ----------
        array : np.ndarray
            Array containing SemanticLiDAR point information.

        Returns
        -------
        Tuple[np.ndarray, np.ndarray]
            Tuple containing points and additional information.
        """
        points = np.array([array['x'], array['y'], array['z']]).T
        points = np.reshape(points, (-1, 3))
        # Add dummy for intensity
        points = np.concatenate((points, np.zeros((points.shape[0], 1))), axis=1)
        # Add zeroes for intensity
        information = array[['CosAngle', 'ObjIdx', 'ObjTag']]
        return points, information

    def _get_color(self, array: np.ndarray):
        """
        Gets the color representation of SemanticLiDAR points.

        Parameters
        ----------
        array : np.ndarray
            Array containing SemanticLiDAR point information.

        Returns
        -------
        np.ndarray
            Color representation of SemanticLiDAR points.
        """
        labels = np.array(self.information['ObjTag'], dtype=np.uint32)
        int_color = ColorConstants.LABEL_COLORS[labels]
        return int_color


class LiDAR_BEV(LiDAR):
    """
    LiDAR_BEV class representing a LiDAR sensor in a Bird's Eye View (BEV) perspective.

    Inherits from LiDAR class.

    Attributes
    ----------
    is_2d : bool
        Flag indicating if the LiDAR sensor is 2D (True for LiDAR_BEV).
    """

    is_2d = True

    def __init__(self, *args, **kwargs):
        """
        Initializes a LiDAR_BEV object.

        Parameters
        ----------
        *args
            Variable length argument list.
        **kwargs
            Arbitrary keyword arguments.
        """
        super().__init__(*args, **kwargs)

    def _render_data(self, array: np.ndarray) -> np.ndarray:
        """
        Renders the LiDAR data in a Bird's Eye View (BEV) perspective.

        Parameters
        ----------
        array : np.ndarray
            Array containing LiDAR point information.

        Returns
        -------
        np.ndarray
            Rendered LiDAR data in BEV perspective.
        """
        lidar_range = 2.0 * float(self.sensor_options['range'])
        lidar_data = np.array(array[:, :2])
        lidar_color = self._get_color(self.information)
        lidar_data *= min(self.display_size) / lidar_range
        lidar_data += (0.5 * self.display_size[0], 0.5 * self.display_size[1])
        lidar_data = np.fabs(lidar_data)
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (self.display_size[0], self.display_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
        lidar_img[tuple(lidar_data.T)] = lidar_color * 255
        lidar_img = np.flip(lidar_img, axis=0)  # Change direction of the image
        return np.ascontiguousarray(lidar_img)

     
class SemanticLiDAR_BEV(SemanticLiDAR):
    """
    SemanticLiDAR_BEV class representing a semantic LiDAR sensor in a Bird's Eye View (BEV) perspective.

    Inherits from SemanticLiDAR class.

    Attributes
    ----------
    is_2d : bool
        Flag indicating if the LiDAR sensor is 2D (True for SemanticLiDAR_BEV).
    """

    is_2d = True

    def __init__(self, *args, **kwargs):
        """
        Initializes a SemanticLiDAR_BEV object.

        Parameters
        ----------
        *args
            Variable length argument list.
        **kwargs
            Arbitrary keyword arguments.

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
        super().__init__(*args, **kwargs)

    def init_sensor(self, transform, attached, sensor_options):
        """
        Initializes the semantic LiDAR sensor.

        Parameters
        ----------
        transform : carla.Transform
            Transform of the sensor.
        attached : carla.Actor
            Actor to which the sensor is attached.
        sensor_options : Dict[str, str]
            Dictionary containing sensor options.

        Returns
        -------
        carla.Actor
            Spawned semantic LiDAR actor.
        """
        lidar_bp = self.world.get_blueprint_library().find(SensorInformation.SemanticLiDAR.blueprint)
        lidar_bp.set_attribute('range', str(SensorInformation.SemanticLiDAR.OPTIONS.range))  # Set default range

        for key in sensor_options:
            lidar_bp.set_attribute(key, sensor_options[key])

        lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
        return lidar

    def _render_data(self, array: np.ndarray) -> np.ndarray:
        """
        Renders the semantic LiDAR data in a Bird's Eye View (BEV) perspective.

        Parameters
        ----------
        array : np.ndarray
            Array containing semantic LiDAR point information.

        Returns
        -------
        np.ndarray
            Rendered semantic LiDAR data in BEV perspective.
        """
        lidar_range = 2.0 * float(self.sensor_options['range'])
        lidar_data = np.array(array[:, :2])
        lidar_colors = self._get_color(self.information)

        lidar_data *= min(self.display_size) / lidar_range
        lidar_data += (0.5 * self.display_size[0], 0.5 * self.display_size[1])
        lidar_data = np.fabs(lidar_data)
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (self.display_size[0], self.display_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
        lidar_img[tuple(lidar_data.T)] = lidar_colors*255
        lidar_img = np.flip(lidar_img, axis=0)  # Change direction of the image
        return np.ascontiguousarray(lidar_img)
    
    
    
class LiDARMerge(LiDAR):
    """
    LiDARMerge class representing a group of LiDAR sensors merged into a single sensor. The main sensor of this class is a IMU sensor. This is used to merge multiple LiDAR sensors into a single sensor.
    
    Inherits from LiDAR class, this is to inherit the setup of the LiDAR sensor which is used when combining multiple LiDAR sensors into a single sensor.
    """
    def __init__(self, sensor_list:List[LiDAR_Sensor], *args, **kwargs):
        assert all([isinstance(x, LiDAR_Sensor) for x in sensor_list]), f"Expected all sensors in sensor_list to be of type 'LiDAR', but got {sensor_list}"
        assert len(sensor_list) > 1, f"Expected sensor_list to have more than 1 sensor, but got {len(sensor_list)}. Use LiDAR class instead if you only have 1 sensor."
        self.sensor_list:List[LiDAR_Sensor] = sensor_list
        super().__init__(*args, **kwargs)
        self.relative_transforms = [relative_transform_matrix(x, self) for x in self.sensor_list]
        
    def init_sensor(self, transform, attached, sensor_options):
        """
        Initializes the IMU sensor.

        Parameters
        ----------
        transform : carla.Transform
            The transform of the IMU sensor
        attached : carla.Actor
            The actor to which the LiDAR sensor is attached.
        sensor_options : Dict[str, str]
            A dictionary containing the sensor options.

        Returns
        -------
        carla.Actor
            The IMU actor.
        """
        self._pps = np.array([x.pps for x in self.sensor_list])
        self._channels = np.array([x.channels for x in self.sensor_list])
        self._rotation_frequency = np.array([x.rotation_frequency for x in self.sensor_list])
        self._fov = np.array([x.fov for x in self.sensor_list])
        self._upper_fov = np.array([x.upper_fov for x in self.sensor_list])
        self._lower_fov = np.array([x.lower_fov for x in self.sensor_list])
        self._sensor_range = np.array([x.sensor_range for x in self.sensor_list])
        self.atmosphere_attenuation_rate = np.mean([x.atmosphere_attenuation_rate for x in self.sensor_list])


        self._point_spacing = calculate_point_spacing(
            np.mean(self.pps),
            np.mean(self.channels),
            np.mean(self.rotation_frequency),
            np.mean(self.fov),
            np.mean(self.upper_fov)+np.mean(self.lower_fov)
        )
        # Let a IMU sensor be the main sensor to which the merged LiDAR sensors uses as a reference.
        imu_blueprint = self.world.get_blueprint_library().find("sensor.other.imu")
        for key, value in sensor_options.items():
            imu_blueprint.set_attribute(key, value)
        imu_sensor = self.world.spawn_actor(imu_blueprint, transform, attach_to=attached)
        return imu_sensor
    
    def _process_data(self, data: carla.IMUMeasurement) -> np.ndarray:
        """
        Processesing the data of the list of LiDAR sensors by combining them into the IMU sensor's frame of reference.

        Parameters
        ----------
        data : carla.LidarMeasurement
            The raw LiDAR data.
        """
        _ = data # Unused variable
        # Get the LiDAR data from each LiDAR sensor. This data is the processed data from each LiDAR sensor. #TODO: Complete the projection into the correct frame of reference.
        xyzi = [x.get_scene() for x in self.sensor_list]

        informations = [x[:,3:] for x in xyzi]
        self.information = np.concatenate(informations, axis=0)
        # transformed_xyz = [transform2target(x[:,:3], self.relative_transforms[i]) for i, x in enumerate(xyzi)]
        w2s = self.w2s()
        transformed_xyz = [world2sensor(sensor2world(x[:,:3], self.sensor_list[i].s2w()),w2s) for i, x in enumerate(xyzi)]
        
        points = np.concatenate([np.hstack([x, i[:, :1]]) for x,i in zip(transformed_xyz, informations)], axis=0)
        return points
    
    def forward_vector(self, numpy: bool = False, normalized: bool = True) -> Union[np.ndarray, List[carla.Vector3D]]:
        """
        Custom overwrite the forward vector to be a forward matrix for each LiDAR sensor.
        
        Parameters
        ----------
        numpy : bool, optional
            Whether to return the forward vector as a numpy array. Defaults to False.
        normalized : bool, optional
            Whether to normalize the forward vector. Defaults to True.
            
        Returns
        -------
        np.ndarray | List[carla.Vector3D]
            The forward vectors of the LiDAR sensors in either a numpy array or a list of carla.Vector3D.
        """
        forward_vectors = [x.forward_vector(numpy=numpy, normalized=normalized) for x in self.sensor_list]
        if numpy:
            return np.array(forward_vectors)
        return forward_vectors
        
    def location(self, numpy: bool = False) -> Union[np.ndarray, List[carla.Location]]:
        """
        Custom overwrite the location to be the location of each LiDAR sensor.
        
        Parameters
        ----------
        numpy : bool, optional
            Whether to return the location as a numpy array. Defaults to False.
            
        Returns
        -------
        np.ndarray | List[carla.Location]
            The locations of the LiDAR sensors in either a numpy array or a list of carla.Location.
        """
        locations = [x.location(numpy=numpy) for x in self.sensor_list]
        if numpy:
            return np.array(locations)
        return locations
        


