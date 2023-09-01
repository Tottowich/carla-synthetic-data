import glob
import os
import sys
import carla
from enum import Enum
from dataclasses import dataclass
import numpy as np
from matplotlib import cm

# Set the root path of the Unreal Engine 4 (UE4) installation
UE4_ROOT = os.environ.get('UE4_ROOT')

class MAPS(Enum):
    # Constants for different CARLA maps
    TOWN01   = "Town01"    # A small, simple town with a river and several bridges.
    TOWN02   = "Town02"    # A small simple town with a mixture of residential and commercial buildings.
    TOWN03   = "Town03"    # A larger, urban map with a roundabout and large junctions.
    TOWN04   = "Town04"    # A small town embedded in the mountains with a special "figure of 8" infinite highway.
    TOWN05   = "Town05"    # Squared-grid town with cross junctions and a bridge. It has multiple lanes per direction. Useful to perform lane changes.
    TOWN06   = "Town06"    # Long many lane highways with many highway entrances and exits. It also has a Michigan left.
    TOWN07   = "Town07"    # A rural environment with narrow roads, corn, barns and hardly any traffic lights.
    TOWN08HD = "Town08HD"  # Secret "unseen" town used for the Leaderboard challenge
    TOWN09HD = "Town09HD"  # Secret "unseen" town used for the Leaderboard challenge
    TOWN10   = "Town10HD"  # A downtown urban environment with skyscrapers, residential buildings and an ocean promenade.
    TOWN11   = "Town11"    # A Large Map that is undecorated. Serves as a proof of concept for the Large Maps feature.
    TOWN12   = "Town12"    # A Large Map with numerous different regions, including high-rise, residential and rural environments.


class PathConstants:
    # Constants for file paths
    CARLA_PATH = glob.glob('/home/thjo/Code/Carla/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))  # Path to the CARLA simulator


@dataclass
class SensorOptions:
    sensor_tick: float = 0.0  # Simulation seconds between sensor captures (ticks).
    def options(self):
        """
        Returns a list of all the keys. These are the names of the options. These are the same as the names of the attributes.
        """
        return [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("__")]
    def as_kwargs(self):
        """
        Returns a dictionary of all the attributes.
        """
        return {attr: getattr(self, attr) for attr in self.options() if (getattr(self, attr) is not None and not attr.startswith("__")) }


class SensorConstants:
    blueprint: str = None
    DATATYPE: type = None
    OPTIONS: SensorOptions = None

class CameraOptions(SensorOptions):
    bloom_intensity: float = 0.675       # Intensity for the bloom post-process effect, 0.0 for disabling it.s
    fov: float = 90.0                    # Horizontal field of view in degrees.
    fstop: float = 1.4                   # Opening of the camera lens. Aperture is 1/fstop with typical lens going down to f/1.2 (larger opening). Larger numbers will reduce the Depth of Field effect.
    image_size_x: int = 800              # Image width in pixels.
    image_size_y: int = 600              # Image height in pixels.
    iso: float = 100.0	                 # The camera sensor sensitivity.
    gamma: float = 2.2                   # The gamma correction to apply to the final image.
    lens_flare_intensity: float = 0.1    # Intensity of the lens flare effect, 0.0 for disabling it.
    sensor_tick: float = 0.0             # Simulation seconds between sensor captures (ticks).
    shutter_speed: float = 200.0         # The camera shutter speed in seconds (1.0/s).
    # Camera lens distortion attributes
    lens_circle_falloff: float = 5.0     # Range: [0.0, 10.0]. Higher values make the lens distortion appear more circular. Lower values make it appear more like a fisheye.
    lens_circle_multiplier: float = 0.0  # Range: [0.0, 10.0]. Higher values make the lens distortion appear more circular. Lower values make it appear more like a fisheye.
    lens_k: float = -1.0                 # Range: [-inf, inf]. The radial distortion coefficient. Smaller values reduce barrel distortion, larger values increase it.
    lens_kcube: float = 0.0              # Range: [-inf, inf]. The tangential distortion coefficient. Smaller values decreases shearing, larger values increase it.

class DepthCameraOptions(CameraOptions):
    far: int = 16777215  # Maximum depth value for the Depth Camera sensor. See https://carla.readthedocs.io/en/latest/ref_sensors/#depth

class SemanticSegmentationCameraOptions(CameraOptions):
    pass 

class InstanceSegmentationCameraOptions(CameraOptions):
    pass 

class LiDAROptions(SensorOptions):
    channels: int = 32                         # Number of channels in the LiDAR sensor.
    range: float = 100.0                       # Maximum range of the LiDAR sensor.
    points_per_second: int = 56000             # Number of points generated by the LiDAR sensor per second.
    rotation_frequency: float = 10.0           # Rotation frequency of the LiDAR sensor in hertz.
    upper_fov: float = 10.0                    # Upper bound of the LiDAR sensor field of view in degrees.
    lower_fov: float = -30.0                   # Lower bound of the LiDAR sensor field of view in degrees.
    horizontal_fov: float = 360.0              # Horizontal field of view in degrees.
    atmosphere_attenuation_rate: float = 0.004 # Coefficient that measures the LIDAR instensity loss per meter. Check the intensity computation above.
    dropoff_general_rate: float = 0.45         # General drop-off rate of LiDAR intensity.
    dropoff_intensity_limit: float = 0.8       # For the intensity based drop-off, the threshold intensity value above which no points are dropped.
    dropoff_zero_intensity: float = 0.4        # For the intensity based drop-off, the probability of each point with zero intensity being dropped.
    pixel_values: tuple = (255, 255, 255)      # Pixel values for LiDAR points visualization.
    noise_stddev: float = 0.0                  # Standard deviation of the noise model for the LiDAR sensor.

class SemanticLiDAROptions(LiDAROptions):
    pass # Not implemented yet

class IMUOptions(SensorOptions):
    noise_accel_stddev_x: float = 0.0  # Standard deviation parameter in the noise model for acceleration (X axis).
    noise_accel_stddev_y: float = 0.0  # Standard deviation parameter in the noise model for acceleration (Y axis).
    noise_accel_stddev_z: float = 0.0  # Standard deviation parameter in the noise model for acceleration (Z axis).
    noise_gyro_bias_x: float = 0.0     # Mean parameter in the noise model for the gyroscope (X axis).
    noise_gyro_bias_y: float = 0.0     # Mean parameter in the noise model for the gyroscope (Y axis).
    noise_gyro_bias_z: float = 0.0     # Mean parameter in the noise model for the gyroscope (Z axis).
    noise_gyro_stddev_x: float = 0.0   # Standard deviation parameter in the noise model for the gyroscope (X axis).
    noise_gyro_stddev_y: float = 0.0   # Standard deviation parameter in the noise model for the gyroscope (Y axis).
    noise_gyro_stddev_z: float = 0.0   # Standard deviation parameter in the noise model for the gyroscope (Z axis).
    noise_seed: int = 0                # Initializer for a pseudorandom number generator.


class GNSSOptions(SensorOptions):
    noise_alt_bias: float   = 0.0  # Mean parameter in the noise model for altitude.
    noise_alt_stddev: float = 0.0  # Standard deviation parameter in the noise model for altitude.
    noise_lat_bias: float   = 0.0  # Mean parameter in the noise model for latitude.
    noise_lat_stddev: float = 0.0  # Standard deviation parameter in the noise model for latitude.
    noise_lon_bias: float   = 0.0  # Mean parameter in the noise model for longitude.
    noise_lon_stddev: float = 0.0  # Standard deviation parameter in the noise model for longitude.
    noise_seed: int         = 0    # Seed for the random number generator used to add noise to the measurements.

class SensorInformation:
    class RGBCamera(SensorConstants):
        blueprint: str = "sensor.camera.rgb"
        DATATYPE: type = carla.ColorConverter.Raw
        OPTIONS: SensorOptions = CameraOptions()

    class SemanticSegmentationCamera(SensorConstants):
        blueprint: str = "sensor.camera.semantic_segmentation"
        DATATYPE: type = carla.ColorConverter.CityScapesPalette
        OPTIONS: SensorOptions = SemanticSegmentationCameraOptions()

    class InstanceSegmentationCamera(SensorConstants):
        blueprint: str = "sensor.camera.instance_segmentation"
        DATATYPE: type = carla.ColorConverter.CityScapesPalette
        OPTIONS: SensorOptions = InstanceSegmentationCameraOptions()

    class DepthCamera(SensorConstants):
        blueprint: str = "sensor.camera.depth"
        DATATYPE: type = carla.ColorConverter.Depth
        OPTIONS: SensorOptions = DepthCameraOptions()

    class LogarithmicDepthCamera(DepthCamera):
        DATATYPE: type = carla.ColorConverter.LogarithmicDepth

    class LiDAR(SensorConstants):
        blueprint: str = "sensor.lidar.ray_cast"
        DATATYPE: type = carla.LidarMeasurement
        OPTIONS: SensorOptions = LiDAROptions()

    class SemanticLiDAR(SensorConstants):
        blueprint: str = "sensor.lidar.ray_cast_semantic"
        DATATYPE: type = carla.LidarMeasurement
        OPTIONS: SensorOptions = SemanticLiDAROptions()

    class IMU(SensorConstants):
        blueprint: str = "sensor.other.imu"
        DATATYPE: type = carla.IMUMeasurement
        OPTIONS: SensorOptions = IMUOptions()

    class GNSS(SensorConstants):
        blueprint: str = "sensor.other.gnss"
        DATATYPE: type = carla.GnssMeasurement
        OPTIONS: SensorOptions = GNSSOptions()

    class Radar(SensorConstants):
        blueprint: str = "sensor.other.radar"
        DATATYPE: type = carla.RadarMeasurement
        OPTIONS: SensorOptions = None
    
    @classmethod
    def get_sensor_options(self,name:str):
        info: SensorConstants = getattr(self, name)
        return info.OPTIONS.as_kwargs()

# namespace carla {
# namespace rpc {

#   enum class CityObjectLabel : uint8_t {
#     None         =    0u,
#     // cityscape labels
#     Roads        =    1u,
#     Sidewalks    =    2u,
#     Buildings    =    3u,
#     Walls        =    4u,
#     Fences       =    5u,
#     Poles        =    6u,
#     TrafficLight =    7u,
#     TrafficSigns =    8u,
#     Vegetation   =    9u,
#     Terrain      =   10u,
#     Sky          =   11u,
#     Pedestrians  =   12u,
#     Rider        =   13u,
#     Car          =   14u,
#     Truck        =   15u,
#     Bus          =   16u,
#     Train        =   17u,
#     Motorcycle   =   18u,
#     Bicycle      =   19u,
#     // custom
#     Static       =   20u,
#     Dynamic      =   21u,
#     Other        =   22u,
#     Water        =   23u,
#     RoadLines    =   24u,
#     Ground       =   25u,
#     Bridge       =   26u,
#     RailTrack    =   27u,
#     GuardRail    =   28u,

#     Any          =  0xFF
#   };

# } // namespace rpc
# } // namespace carla

class ColorConstants:
    VIRIDIS = np.array(cm.get_cmap('plasma').colors)
    VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
    LABEL_COLORS = np.array([
        (0, 0, 0),         # unlabeled
        (128, 64, 128),    # road
        (244, 35, 232),    # sidewalk
        (70, 70, 70),      # building
        (102, 102, 156),   # wall
        (190, 153, 153),   # fence
        (153, 153, 153),   # pole
        (250, 170, 30),    # traffic light
        (220, 220, 0),     # traffic sign
        (107, 142, 35),    # vegetation
        (152, 251, 152),   # terrain
        (70, 130, 180),    # sky
        (220, 20, 60),     # pedestrian
        (255, 0, 0),       # rider
        (0, 0, 142),       # Car
        (0, 0, 70),        # truck
        (0, 60, 100),      # bus
        (0, 80, 100),      # train
        (0, 0, 230),       # motorcycle
        (119, 11, 32),     # bicycle
        (110, 190, 160),   # static
        (170, 120, 50),    # dynamic
        (55, 90, 80),      # other
        (45, 60, 150),     # water
        (157, 234, 50),    # road line
        (81, 0, 81),       # ground
        (150, 100, 100),   # bridge
        (230, 150, 140),   # rail track
        (180, 165, 180),   # guard rail
    ]) / 255.0 # normalize each channel [0-1] since is what Open3D uses
 
     