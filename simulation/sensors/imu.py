
from simulation.sensors.sensor import Sensor, SensorCallback
import carla
from typing import Dict, List, Tuple
import numpy as np

class IMU(Sensor):
    """
    IMU (Inertial Measurement Unit) sensor class for Carla simulator.
    Inherits from the Sensor class.
    """

    is_2d = False

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
        *args,
        **kwargs,
    ):
        super().__init__(
            world,
            transform,
            attached,
            sensor_options,
            verbose,
            display_manager,
            display_pos,
            callbacks,
        )

    def init_sensor(self, transform, attached, sensor_options) -> carla.Sensor:
        """
        Initializes the IMU sensor object.

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
            The initialized IMU sensor object.
        """
        imu_blueprint = self.world.get_blueprint_library().find("sensor.other.imu")
        for key, value in sensor_options.items():
            imu_blueprint.set_attribute(key, value)
        imu_sensor = self.world.spawn_actor(imu_blueprint, transform, attach_to=attached)
        return imu_sensor

    def _render_data(self, array: np.ndarray) -> np.ndarray:
        """
        Renders the IMU sensor data.

        Parameters
        ----------
        array : np.ndarray
            The processed IMU sensor data.

        Returns
        -------
        np.ndarray
            The rendered scene.
        """
        # IMU sensor does not have a rendering function
        return array

    def _process_data(self, data: carla.SensorData) -> np.ndarray:
        """
        Processes the IMU sensor data.

        Parameters
        ----------
        data : carla.SensorData
            The IMU sensor data.

        Returns
        -------
        np.ndarray
            The processed data.
        """
        print(f"Processing IMU data: {data}")
        imu_data = data  # Process IMU data here
        processed = np.array([imu_data.accelerometer, imu_data.gyroscope])
        return processed

