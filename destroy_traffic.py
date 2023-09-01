import carla
from utils.carla_objects import CarlaObjects
from simulation.environment.actor_manager import CarlaActorManager



client = carla.Client('localhost', 2000)
world = client.get_world()

library = world.get_blueprint_library()
custom_object = CarlaObjects.get_all_objects()

ca_manager = CarlaActorManager(client=client,world=world,objects=custom_object)

ca_manager.destroy_all()
# Destroy all vehicles