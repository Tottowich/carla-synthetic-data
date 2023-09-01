

import argparse
import random
import time

import carla

from simulation.environment.actor_manager import CarlaActorManager
from simulation.environment.traffic import CarlaTrafficSpawner
from simulation.sensors.cameras import RGBCamera
from simulation.sensors.lidars import (LiDAR, LiDAR_BEV, SemanticLiDAR,
                                       SemanticLiDAR_BEV)
from simulation.sensors.sensor_callbacks import (BoundingBoxOriented,
                                                 BoundingBoxProjector)
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

    bp = world.get_blueprint_library().filter('charger_2020')[0]
    # Spawn a vehicle and set it to autopilot

    # Traffic and enviroment setup
    traffic_spawner = CarlaTrafficSpawner(client, world)
    traffic_spawner.spawn_traffic(num_vehicles=100, num_pedestrians=200)
    traffic_spawner.start_traffic()

    # Warm up the simulation
    warm_up(world)

    # Initialize the PointCloudVisualizer and DisplayManager
    pcd_visualizer = PointCloudVisualizer(display_size=(args.width, args.height), point_size=2)
    display_manager = DisplayManager(grid_size=[2, 3], window_size=[args.width, args.height], display_index=1)
    
    # Create the actor manager, which will be used to locate all wanted actors
    ca_manager = CarlaActorManager(client, world, [CarlaObjectCategories.Vehicles, CarlaObjects.TrafficLight, CarlaObjects.TrafficSigns, CarlaObjectCategories.Pedestrians])
    # Spawn ego vehicle
    tries = 0
    while vehicle is None or tries < 50:
        try:
            vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
            vehicle.set_autopilot(True)
        except:
            tries += 1
            continue
    # Create LiDAR, RGB cameras, and LiDAR_BEV sensors
    sensor_rotation = carla.Rotation(yaw=0, pitch=0, roll=0)
    lidar = LiDAR(world=world, transform=carla.Transform(carla.Location(x=0, z=2.4), sensor_rotation),
                    attached=vehicle, 
                    sensor_options={'channels': 128,
                                    'range': 50,
                                    'points_per_second': 500000,
                                    'rotation_frequency': 20,
                                    },
                    display_manager=pcd_visualizer, display_pos=[0, 0],
                    callbacks=[BoundingBoxOriented(ca_manager.actors)])
    
    rgb_front = RGBCamera(world=world, transform=carla.Transform(carla.Location(x=0, z=2.4), sensor_rotation),
                         attached=vehicle, sensor_options={'fov': 120}, display_manager=display_manager,
                         display_pos=[0, 1], callbacks=[BoundingBoxProjector(ca_manager.actors)])
    rgb_left = RGBCamera(world=world, transform=carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-120, pitch=0, roll=0)),
                         attached=vehicle, sensor_options={'fov': 120}, display_manager=display_manager,
                         display_pos=[0, 0], callbacks=[BoundingBoxProjector(ca_manager.actors)])
    rgb_right = RGBCamera(world=world, transform=carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=120, pitch=0, roll=0)),
                            attached=vehicle, sensor_options={'fov': 120}, display_manager=display_manager,
                            display_pos=[0, 2], callbacks=[BoundingBoxProjector(ca_manager.actors)])
    lidar_bev = LiDAR_BEV(world=world, transform=carla.Transform(carla.Location(x=0, z=2.4)),
                            attached=vehicle,
                            sensor_options={'channels': 128,
                                            'range': 20,
                                            'points_per_second': 500000,
                                            'rotation_frequency': 20,
                                            }, display_manager=display_manager,
                            display_pos=[1, 0])
    semantic_lidar_bev = SemanticLiDAR_BEV(world=world, transform=carla.Transform(carla.Location(x=0, z=2.4)),
                                            attached=vehicle,
                                            sensor_options={'channels': 128,
                                                            'range': 20,
                                                            'points_per_second': 500000,
                                                            'rotation_frequency': 20,
                                                            }, display_manager=display_manager,
                                            display_pos=[1, 2])
    if display_manager:
        display_manager.start()

    while is_running(display_manager, pcd_visualizer):

        if display_manager:
            display_manager.check_events()
        if pcd_visualizer:
            pcd_visualizer.render()

        if args.sync:
            world.tick()
        else:
            world.wait_for_tick()
        
    if pcd_visualizer:
        pcd_visualizer.stop()
    if display_manager:
        display_manager.stop()
    world.apply_settings(original_settings)
    if vehicle is not None:
        vehicle.destroy()
    ca_manager.destroy_all()
    # Restore the original settings of the world
    # Destroy the vehicle


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

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)
        run_simulation(args, client)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
