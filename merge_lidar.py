

import argparse
import random
import time

import carla

from simulation.environment.actor_manager import CarlaActorManager
from simulation.environment.traffic import CarlaTrafficSpawner
from simulation.sensors.cameras import RGBCamera
from simulation.sensors.lidars import (LiDAR, LiDARMerge)
from simulation.sensors.sensor_callbacks import (BoundingBoxMultiSensor, SceneExporter)
from utils.carla_objects import CarlaObjectCategories, CarlaObjects
from utils.helper import CustomTimer, destroy_all_vehicles
from visualization.display_manager import DisplayManager
from visualization.point_cloud_visualizer import PointCloudVisualizer
from utils.initializations import warm_up
from utils.helper import carla_vec2np_array


def run_simulation(args, client):
    """
    Run a simulation using the given arguments and Carla client.

    Parameters
    ----------
    args : argparse.Namespace
        The arguments to use for the simulation.
    client : carla.Client
        The Carla client to use for the simulation.
    """

    display_manager = None
    vehicle = None
    timer = CustomTimer()

    world = client.get_world()  # Get the Carla world
    original_settings = world.get_settings()  # Get the original settings of the world
    destroy_all_vehicles(world)

    if args.sync:
        # Enable synchronous mode if specified
        traffic_manager = client.get_trafficmanager(8000)
        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        settings.no_rendering_mode = not args.render
        world.apply_settings(settings)


    # Traffic and enviroment setup
    traffic_spawner = CarlaTrafficSpawner(client, world)
    traffic_spawner.spawn_traffic(num_vehicles=100, num_pedestrians=200)
    traffic_spawner.start_traffic()
    warm_up(world, ticks=100)
    ca_manager = CarlaActorManager(client, world, [CarlaObjects.NonDrivable])
    
    # Spawn ego vehicle
    bp = world.get_blueprint_library().filter('charger_2020')[0]
    tries = 0
    while vehicle is None or tries < 50:
        try:
            vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
            vehicle.set_autopilot(True)
        except:
            tries += 1
            continue

    # Warm up the simulation

    # Initialize the PointCloudVisualizer and DisplayManager
    pcd_visualizer = PointCloudVisualizer(display_size=(args.width, args.height), point_size=2)
    display_manager = None #DisplayManager(grid_size=[2, 3], window_size=[args.width, args.height], display_index=1)
    
    # Create the actor manager, which will be used to locate all wanted actors
    # Spawn ego vehicle
    # Get vehicle dimensions
    vehicle_dim = vehicle.bounding_box.extent
    length = 2*vehicle_dim.x
    width = 2*vehicle_dim.y
    height = 2*vehicle_dim.z
    
    # Create LiDAR, RGB cameras, and LiDAR_BEV sensors
    pos_front = carla.Location(x=length/2, z=height/4)
    pos_left  = carla.Location(x=length/4,y=-width/2, z=height/3)
    pos_back  = carla.Location(x=-length/2, z=height/4)
    pos_right = carla.Location(x=length/4,y=width/2, z=height/3)
    transform_front = carla.Transform(pos_front, carla.Rotation(yaw=0, pitch=0, roll=0))
    transform_left  = carla.Transform(pos_left, carla.Rotation(yaw=-90, pitch=0, roll=0))
    transform_back  = carla.Transform(pos_back, carla.Rotation(yaw=+180, pitch=0, roll=0))
    transform_right = carla.Transform(pos_right, carla.Rotation(yaw=+90, pitch=0, roll=0))
    
    sensor_options = {
        'channels': 128,
        'range': 25,
        'points_per_second': 104e3,
        'rotation_frequency': 20,
        'horizontal_fov': 130,
        'upper_fov': 0,
        'lower_fov': -80,
    }
    
    sensor_rotation = carla.Rotation(yaw=0, pitch=0, roll=0)
    
    lidar_front= LiDAR(world=world, transform=transform_front,
                    attached=vehicle, 
                    sensor_options=sensor_options, verbose=True)
    
    lidar_back = LiDAR(world=world, transform=transform_back,
                    attached=vehicle, 
                    sensor_options=sensor_options, verbose=True)
    
    lidar_left = LiDAR(world=world, transform=transform_left,
                    attached=vehicle, 
                    sensor_options=sensor_options, verbose=True)
    lidar_right = LiDAR(world=world, transform=transform_right,
                    attached=vehicle, 
                    sensor_options=sensor_options, verbose=True)
    
    lidar_merge = LiDARMerge(sensor_list=[lidar_front,lidar_left,lidar_back,lidar_right],world=world,
                    transform=carla.Transform(carla.Location(x=0, z=1.0), sensor_rotation),
                    attached=vehicle, verbose=True,
                    display_manager=pcd_visualizer,
                    callbacks=[BoundingBoxMultiSensor(ca_manager.actors, visualize=True)]) #SceneExporter(ca_manager.actors,visualize=True)
    if display_manager:
        display_manager.start()
    try:
        while is_running(display_manager, pcd_visualizer):

            if display_manager:
                display_manager.check_events()
            if pcd_visualizer:
                pcd_visualizer.render()

            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()
    except KeyboardInterrupt:
        print('Cancelled by user. Bye!')
    finally:
        if pcd_visualizer:
            pcd_visualizer.stop()
        if display_manager:
            display_manager.stop()
        world.apply_settings(original_settings)
        if vehicle is not None:
            vehicle.destroy()
        ca_manager.destroy_all()


def is_running(display_manager:DisplayManager, pcd_visualizer:PointCloudVisualizer):
    """
    Check if the simulation should continue running.

    Parameters
    ----------
    display_manager : DisplayManager
        The display manager to check.
    pcd_visualizer : PointCloudVisualizer
        The point cloud visualizer to check.
    
    Returns
    -------
    bool
        True if the simulation should continue running, False otherwise.
    """
    disp_running = True
    pcd_running = True

    if display_manager is not None:
        disp_running = display_manager.is_running()
    if pcd_visualizer is not None:
        pcd_running = pcd_visualizer.is_running()

    return disp_running and pcd_running


def main():
    argparser = argparse.ArgumentParser(description='CARLA Sensor tutorial')
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sync', action='store_true', help='Synchronous mode execution')
    argparser.add_argument('--async', dest='sync', action='store_false', help='Asynchronous mode execution')
    argparser.set_defaults(sync=True)
    argparser.add_argument('--res', metavar='WIDTHxHEIGHT', default='1280x720', help='window resolution (default: 1280x720)')
    argparser.add_argument('--no-render', dest='render', action='store_false', help='no render mode (default: False)')
    argparser.set_defaults(render=True)
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)
    run_simulation(args, client)


if __name__ == '__main__':
    main()
