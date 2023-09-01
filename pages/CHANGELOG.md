


# Changelog 📜📅🔄

### [2023-07-03]📅
- **Added 📥:** **BoundingBoxOriented** Is a SensorCallback which projects **all visible** bounding boxes into the sensor frame of reference given a LiDAR sensor. This callback is currently only available for LiDAR subclasses. The class can be found in the [simulation/sensors_callbacks.py](../simulation/sensors_callbacks.py) file. The class works with the **PointCloudVisualizer** class and displays the bounding boxes as [Open3D OrientedBoundingBoxes](http://www.open3d.org/docs/latest/python_api/open3d.geometry.OrientedBoundingBox.html) in the point cloud data.
- **Added 📥:** **CarlaObject** used as a wrapper for [carla.CityObjectLabel](https://carla.readthedocs.io/en/latest/python_api/#carlacityobjectlabel). The class can be found in the [utils/carla_objects.py](../utils/carla_objects.py) file.
- **Added 📥:** **CarlaObjects** a container for all **CarlaObject** instances. The class can be found in the [utils/carla_objects.py](../utils/carla_objects.py) file. It is used to store all **CarlaObject** instances in a single object for easy access. When creating a custom type of **CarlaObject** it is important to add the new type to the **CarlaObjects** class.
- **Added 📥:** **CarlaObjectCategory** contains a group of **CarlaObject** instances. The class can be found in the [utils/carla_objects.py](../utils/carla_objects.py) file. There are some predefined categories available.
- **Added 📥:** **CarlaObjectCategories** a container for all **CarlaObjectCategory** instances. The class can be found in the [utils/carla_objects.py](../utils/carla_objects.py) file. It is used to store all **CarlaObjectCategory** instances in a single object for easy access.
- **Added 📥:** Object coloring during visualization. Each color can be found and edited in the [utils/constants.py](../utils/carla_objects.py) file. The colors are stored in each **CarlaObject** instance. Easily accessed through the *dataclass* **CarlaObjects**.
- **Added 📥:** **CarlaActor** which is a wrapper for [carla.Actor](https://carla.readthedocs.io/en/latest/python_api/#carlaactor). The class can be found in the [simulation.environment.carla_actor.py](../simulation/environment/carla_actor.py) file. An instance can be created from either a [carla.Actor](https://carla.readthedocs.io/en/latest/python_api/#carlaactor) or a [carla.EnvironmentObject](https://carla.readthedocs.io/en/latest/python_api/#EnvironmentObject). This solves an issue where the [carla.Actor](https://carla.readthedocs.io/en/latest/python_api/#carlaactor) and [carla.EnvironmentObject](https://carla.readthedocs.io/en/latest/python_api/#EnvironmentObject) does not contain similar attributes. This is to be able to keep track of all objects in the simulation.
- **Added 📥:** **StationaryActor** dummy class to wrapp the [carla.EnvironmentObject](https://carla.readthedocs.io/en/latest/python_api/#EnvironmentObject) class to give it similar attributes as the [carla.Actor](https://carla.readthedocs.io/en/latest/python_api/#carlaactor) class. The class can be found in the [simulation.environment.carla_actor.py](../simulation/environment/carla_actor.py) file. This is to be able to keep track of all objects in the simulation.
- **Added 📥:** **CarlaTrafficSpawner** which can spawn an arbitrary number of vehicles and pedestrians in the simulation. The class can be found in the [simulation/environment/traffic.py](../simulation/environment/traffic.py) file.
- **Added 📥:** **CarlaActorManager** which can read all spawned or default objects and create a **CarlaActor** for each instance with the correct **CarlaObject** label.
- **Removed 🗑️:** **BoundingBoxOpen3D** has been removed and replaced by **BoundingBoxOriented**.
- **Performance 🚀:** **BoundingBoxProjector** and it's subclasses now initalizes all bounding boxes based on a list of **CarlaActors**. It is usefull an easy to retrieve this list using a **CarlaActorManager**.
- **Fixed 🔧:** Fixed a bug in [multi_3D.py](../multi_3D.py) where the frames would lag behind. This was due to a world.tick() before the rendering of the current frame. Changing order reduced the amount of frames droped due to latency.

### [2023-06-27]📅
- **Added 📥:** **BoundingBoxOpen3D** a SensorCallback which projects **all visible** bounding boxes onto the point cloud data. This callback is currently only available for LiDAR subclasses. The class can be found in the [simulation/sensors_callbacks.py](../simulation/sensors_callbacks.py) file. The class works with the **PointCloudVisualizer** class and displays the bounding boxes as points in the point cloud data.
- **Added 📥:** **LiDAR_Open3D** SensorCallback to all 3D LiDAR sensors to convert the scene of the LiDAR object to an Open3D point cloud. The class can be found in the [simulation/lidars.py](../simulation/lidars.py) file.
- **Added 📥:** Properties to the **Sensor** class;
  - **is_synched** - Boolean value indicating if the sensor is visualized or not.
  - **available_options** - List of all available options for the sensor.
  - **default_options** - List of all default options for the sensor.
  - **sensor_range** - The range of the sensor.
  - **frame_id** - The frame id of the sensor.
- **Added 📥:** *classmethod* **SensorInformation.get_sensor_options** to get the available options for a sensor. The method can be found in the [utils/constants.py](../utils/constants.py) file.
- **Changed 🔄:** **Sensor** structure to utilize a python *Queue* to keep the sensors and visualizers thread safe. The sensor now adds the data to the queue and the visualizer retrieves the data from the queue. This allows for multiple visualizers to be used at once. The **DisplayManager** class can be found in the [visualization/display_manager.py](../visualization/display_manager.py) file.
- **Fixed 🔧:** Fixed a bug where frames would create backlog in the visualization face creating small lag spikes and unsynchronized frames between sensors and world data. Now skips frames if the visualization is not up to date with the world data.

### [2023-06-22]📅
- **Added 📥:** Added **PointCloudVisualizer** class for visualizining point cloud data in 3D. The class can be found in the [visualization/point_cloud_visualizer.py](../visualization/point_cloud_visualizer.py) file.
- **Added 📥:** LiDAR sensor now includes the intensity of the point cloud data in its visualization, both 2D and 3D. The intensity is visualized as a color gradient from blue to red. The intensity is also included in the point cloud data. The intensity is stored in the 4th channel of the point cloud data.
- **Added 📥:** **SensorCallback** class for post processing of sensor data before rendering. The class can be found in the [simulation/sensors.py](../simulation/sensors.py) file.
- **Added 📥:** **BoundingBoxProjector** a SensorCallback which projects **all visible** bounding boxes onto the sensor image. This callback is currently only available for Camera subclasses. The class can be found in the [simulation/sensors_callbacks.py](../simulation/sensors_callbacks.py) file.
- **Added 📥:**  [docs/docstring_style.py](../docs/docstring_style.py) showcasing the standard docstring style for the project.
- **Changed 🔄:** The **Sensor** class now takes a list of **SensorCallback** objects as an argument. The callbacks are called in the order they are given. The **Sensor** class can be found in the [simulation/sensors.py](../simulation/sensors.py) file.
- **Refactoring 📦:** Altered the relation between the **Camera** class and its subclasses. The subclasses now initialize by sending a blueprint and data format parameter to the **Camera** class. The **Camera** class then creates the sensor and assigns the data format. The **Camera** class can be found in the [simulation/sensors.py](../simulation/sensors.py) file.
- **Style 🎨:** Added a consitent class and function documenation style for the entire project. The style can be found in the [utils/docstring_style.py](../utils/docstring_style.py) file. The style is based on the [Numpy docstring style](https://numpydoc.readthedocs.io/en/latest/format.html#docstring-standard).
- **Performance 🚀:** The **DisplayManager** renders now operates in a seperate thread. This inceases efficiency and allows for several visualizers at once. This also allows for the **DisplayManager** to be used in a non-blocking manner.
- **Fixed 🔧:** Fixed a bug where the **DisplayManager** would render the raw image data instead of the post processed image data. This bug was caused by the **DisplayManager** not waiting for the sensor to finish processing the data before rendering it and simply retieving the **scene** attribute of the camera sensor. This bug is now fixed by only assigning the **scene** attribute of the camera sensor when the sensor has finished processing the data.
- **Fixed 🔧:** Fixed a bug where the **DisplayManager** would not render the bounding boxes of the **BoundingBoxProjector** callback. This bug was caused by the **DisplayManager** not waiting for the sensor to finish processing the data before rendering it and simply retieving the **scene** attribute of the camera sensor. This bug is now fixed by only assigning the **scene** attribute of the camera sensor when the sensor has finished processing the data.
- **Maintenance 🧹:** Cleaned up the codebase and removed unused variables and imports. Also cleaned the [utils/constants.py](../utils/constants.py) file and removed unused constants.


### [2023-06-19]📅
- **Added 📥:** Added a new class called **SensorOptions** which contains all the available options for a sensor. The class can be found in the [utils/constants.py](../utils/constants.py) file.
- **Changed 🔄:** Altered the structure of the constants regarding sensor information. The constants can be found in the [utils/constants.py](../utils/constants.py) file. Each sensor has its own class with the constants for that sensor. The constants now contain an 'OPTIONS' attribute which contains all available CARLA blueprint options and there default value. See the **SensorOptions** class in [utils/constants.py](../utils/constants.py).
- **Performance 🚀:** Optimized the code for the **DisplayManager** and **Sensor** classes. The **DisplayManager** class now only updates the display when the sensor has new data. The **Sensor** class now only updates the sensor when the sensor has new data. This results in a significant performance boost.


### [2023-06-17]📅
- **Added 📥:** Sensor wrappers for RGB Camera, Depth Camera, Semantic Segmentation Camera, LiDAR, Semantic LiDAR. The classes can be found in the [simulation/sensors.py](../simulation/sensors.py)
- **Added 📥:** Modified visualization of all the sensors to be more modular. The visualizer was based on the example visualizer found in the [CARLA repository](https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/visualize_multiple_sensors.py)
- **Added 📥:** Added traffic generation using example script from CARLA repository found [here](https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/generate_traffic.py)






## Categories
### [Date]📅
| Category                | Example description                                |
|-------------------------|----------------------------------------------------|
| **Added 📥**            | Added new feature X                                |
| **Changed 🔄**          | Updated function Y to improve performance          |
| **Removed 🗑️**          | Removed deprecated API Z                           |
| **Fixed 🔧**            | Fixed bug causing application crash                |
| **Security 🔒**         | Implemented enhanced encryption for user data      |
| **Deprecated 📛**       | Marked method A as deprecated, use method B instead|
| **Breaking Changes 🚨** | Renamed class C, update references accordingly     |
| **Documentation 📚**    | Updated API documentation for better clarity       |
| **Maintenance 🧹**      | Cleaned up codebase, removed unused variables      |
| **Performance 🚀**      | Optimized database queries for faster response time|
| **Refactoring 📦**      | Extracted reusable components from module D        |
| **Style 🎨**            | Applied consistent coding style across the project |
| **Tests 🧪**            | Added unit tests for module E                      |
| **Other 📦**            | Miscellaneous updates and improvements             |

<details close>

<summary>👉 Example of a changelog</summary>

### [2023-06-01]📅 - Title
- **Added 📥:** Implemented data generation using CARLA simulator
- **Fixed 🔧:** Fixed bug causing application crash

Summary of changes made in this version.

</details>