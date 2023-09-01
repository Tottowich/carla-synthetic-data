from utils.carla_objects import CarlaObject, CarlaObjectCategory, blueprint2carla_object
from simulation.environment.carla_actor import CarlaActor
from typing import Union, List, Tuple
import carla


class CarlaActorManager:
    """
    Given a list of CarlaObjects or CarlaObjectCategory, locate the corresponding actors in the Carla world and create a list of CarlaActor.
    The CarlaActorManager can also update the list of CarlaActor with new CarlaObjects or CarlaObjectCategory.
    The CarlaActorManager can also destroy all the actors in the list.
    
    Parameters
    ----------
    client : carla.Client
        The Carla client
    world : carla.World
        The Carla world
    objects : List[Union[CarlaObject, CarlaObjectCategory]]
        The list of available CarlaObjects or CarlaObjectCategory in the Carla world.
    """
    def __init__(self, client:carla.Client, world: carla.World, objects: List[Union[CarlaObject, CarlaObjectCategory]]):
        self.client = client
        self.world = world
        self.objects = None
        self.actors:List[CarlaActor] = None
        self.update(objects, overwrite=True)
        
        self.blueprint_library = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        
    
    def _get_environment_objects(self, objects: List[Union[CarlaObject, CarlaObjectCategory]]):
        """
        Get the environment objects from the list of objects to spawn.
        
        Returns
        -------
        List[CarlaActor]
            The list of CarlaActor corresponding to the environment objects. These actors are not spawned into the carla world.
            
            They are present by default in the Carla world.
        """
        actors = []
        for carla_object in objects:
            if isinstance(carla_object, CarlaObjectCategory):
                # Recursively get the objects from the category
                actors.extend(self._get_environment_objects(carla_object))
            else:
                env_objs = self.world.get_environment_objects(carla_object.carla_type)
                for env_obj in env_objs:
                    actors.append(CarlaActor(env_obj, carla_object))
        print(f"Found {len(actors)} environment objects from {objects}")
        return actors
    
    def _get_live_actors(self, objects: List[Union[CarlaObject, CarlaObjectCategory]]):
        """
        Get the None environment objects from the list of objects to spawn.

        Parameters
        ----------
        objects :  List[Union[CarlaObject, CarlaObjectCategory]]

        Returns
        -------
        List[CarlaActor]
            The list of CarlaActor corresponding to the live actors in the Carla world. These actors have been spawned into the carla world.
            
            They are not present by default in the Carla world.

        """
        actors = []
        for carla_object in objects:
            if isinstance(carla_object, CarlaObjectCategory):
                # Recursively get the objects from the category
                actors.extend(self._get_live_actors(carla_object))
            else:
                for blueprint_id in carla_object.blueprint_ids:
                    live_actors = self.world.get_actors().filter(blueprint_id)
                    for live_actor in live_actors:
                        actors.append(CarlaActor(live_actor, carla_object))
        print(f"Found {len(actors)} live actors from {objects}")
        return actors

    def keep_unique(self):
        """
        Keep only unique actors in the actors list.

        Returns
        -------
        None. The actors are updated in place.
        """
        self.actors = list(set(self.actors))
    def update(self, objects:List[Union[CarlaObject, CarlaObjectCategory]], overwrite:bool=False):
        """
        Update the actors.

        Parameters
        ----------
        objects : List[Union[CarlaObject, CarlaObjectCategory]]
            The list of objects to track.
        overwrite : bool, optional
            If True, the currnent actors list is overwritten. Otherwise, the actors are added to the list. The default is False.
        """
        if overwrite:
            self.objects = []
            self.actors = []
        self.objects.extend(objects)
        self.actors = []
        self.actors.extend(self._get_environment_objects(self.objects))
        self.actors.extend(self._get_live_actors(self.objects))
        self.keep_unique()
    def destroy_all(self):
        """
        Destroy all actors.

        Returns
        -------
        None. The actors are destroyed in place.
        """
        self.client.apply_batch([carla.command.DestroyActor(x.actor) for x in self.actors if (x.actor is not None and isinstance(x.actor, carla.Actor))])

        # for actor in self.actors:
        #     actor.destroy()
        print(f"Destroyed {len(self.actors)} actors")
        self.actors = []
