import random
from dataclasses import dataclass
from typing import List, Tuple, Union

import carla
import matplotlib.cm as cm
import numpy as np

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
# } // namespace carla # This file can be found in <path/to/carla>/LibCarla/source/rpc/ObjectLabel.h

def try_spawn(world:carla.World, blueprint:carla.ActorBlueprint, transform:carla.Transform=None, attempts:int=20):
    """
    Try to spawn the actor into the world. If the spawn fails, return None.
    
    Parameters
    ----------
    world : carla.World
        The world to spawn the actor into.
    blueprint : carla.ActorBlueprint
        The blueprint of the actor to spawn.
    transform : carla.Transform
        The transform of the actor to spawn.
    
    Returns
    -------
    carla.Actor
        The spawned actor. If the spawn fails, return None.
    """
    while attempts > 0:
        if transform is None:
            transform = random.choice(world.get_map().get_spawn_points())
        try:
            actor = world.spawn_actor(blueprint, transform)
            return actor
        except:
            attempts -= 1
            continue

@dataclass
class CarlaObject:
    """Represents a Carla city object with an ID, color, and Carla type.
    
    Parameters/Attributes
    ----------
    id : int
        The ID of the object typ.
    color : np.ndarray
        The RGB color of the object.
    carla_type : carla.CityObjectLabel
        The Carla type of the object.
    blueprints : list
        A list of carla.ActorBlueprint instances. Default is None.
    """
    id: int
    color: np.ndarray
    carla_type: carla.CityObjectLabel
    blueprint_ids: List[str] = None
    def __init__(self, id: int, color: np.ndarray, carla_type: carla.CityObjectLabel, blueprint_ids: List[str] = None):
        self.id = id
        self.color = np.array(color)
        self.carla_type = carla_type
        self.blueprint_ids = blueprint_ids if blueprint_ids is not None else []
    def __hash__(self):
        return hash(self.id)
    def __eq__(self, other):
        assert isinstance(other, CarlaObject), f"Expected other to be of type {CarlaObject}, but got {type(other)}"
        return self.id == other.id
    def __str__(self) -> str:
        return f"CarlaObject({self.id}, {self.color}, {self.carla_type})"
    @property
    def name(self):
        return self.carla_type.name
    def spawn(self, world: carla.World, quantity:int, blueprint_id:str=None) -> carla.Actor:
        """
        Spawn the object into the world.
        
        Parameters
        ----------
        world : carla.World
            The world to spawn the object into.
        quantity : int
            The number of objects to spawn.
        blueprint_id : str
            The blueprint ID of the object to spawn. Default is None. If None, a random blueprint ID is chosen.
        
        Returns
        -------
        carla.Actor
            The spawned actor.
        """
        if self.name == "Pedestrians":
            return self.spawn_pedestrian(world)
            
        spawned_actors = []
        for _ in range(quantity):
            if blueprint_id is None:
                blueprint_id = random.choice(self.blueprint_ids)
            blueprint = world.get_blueprint_library().find(blueprint_id)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('is_invincible'):
                blueprint.set_attribute('is_invincible', 'false')
            if blueprint.has_attribute('role_name'):
                blueprint.set_attribute('role_name', 'autopilot')
            # Spawn the actor
            actor = try_spawn(world, blueprint)
            if actor is not None:
                spawned_actors.append(actor)
        return spawned_actors


    def spawn_pedestrian(self, world: carla.World, location: carla.Location=None, rotation: carla.Rotation=None) -> carla.Actor:
        """
        Spawn the object into the world.
        
        Parameters
        ----------
        world : carla.World
            The world to spawn the object into.
        location : carla.Location
            The location to spawn the object at.
        rotation : carla.Rotation
            The rotation of the object.
        
        Returns
        -------
        carla.Actor
            The spawned actor.
        """
        PEDESTRIANS_RUNNING  = 0.0     # how many pedestrians will run
        PEDESTRIANS_CROSSING = 0.0     # how many pedestrians will walk through the road
        if rotation is None:
            rotation = carla.Rotation(0, 0, 0)
        if location is None:
            location = random.choice(world.get_map().get_spawn_points())
        blueprint = world.get_blueprint_library().find(random.choice(self.blueprint_ids))
        return world.spawn_actor(blueprint, carla.Transform(location, rotation))
@dataclass
class CarlaObjectCategory:
    """Represents a category of Carla city objects with a name, color, Carla type, and optional children."""
    name: str
    color: np.ndarray
    children: List["CarlaObject"] = None
    
    def __init__(self, name: str, color: np.ndarray,
                 children: List["CarlaObject"]):
        """
        Initialize a CarlaCityObjectCategory instance.
        
        Parameters
        ----------
            name : str
                The name of the category.
            color : tuple
                The RGB color of the category.
            carla_type : carla.CityObjectLabel
                The Carla type of the category.
            children : list
                A list of child CarlaCityObject instances. Default is None.
        """
        self.name = name
        self.color = np.array(color)
        self.children = children
        
    def __iter__(self):
        """
        Iterate over the category and its children.
        """
        return iter(self.children)
    def get_blueprints(self)->List[Tuple[CarlaObject, List[carla.ActorBlueprint]]]:
        """
        Get the blueprints of the category and its children.
        
        Returns
        -------
        list
            A list of carla.ActorBlueprint instances.
        """
        blueprints = []
        for child in self.children:
            blueprints.append((child, child.blueprint_ids))
        return blueprints
    def __str__(self):
        return f"CarlaObjectCategory(name={self.name}, color={self.color}, children={[x.name for x in self.children]})"
    def __repr__(self):
        return self.__str__()
class CarlaObjects:
    """Stores predefined Carla city object labels."""
    NoneLabel = CarlaObject(0, (0, 0, 0), carla.CityObjectLabel.NONE)         # unlabeled
    Roads = CarlaObject(1, (128, 64, 128), carla.CityObjectLabel.Roads)        # road
    Sidewalks = CarlaObject(2, (244, 35, 232), carla.CityObjectLabel.Sidewalks)    # sidewalk
    Buildings = CarlaObject(3, (70, 70, 70), carla.CityObjectLabel.Buildings)      # building
    Walls = CarlaObject(4, (102, 102, 156), carla.CityObjectLabel.Walls)       # wall
    Fences = CarlaObject(5, (190, 153, 153), carla.CityObjectLabel.Fences)      # fence
    Poles = CarlaObject(6, (153, 153, 153), carla.CityObjectLabel.Poles)       # pole
    TrafficLight = CarlaObject(7, (250, 170, 30), carla.CityObjectLabel.TrafficLight,[
        'static.prop.trafficlight',
    ]) # traffic light
    TrafficSigns = CarlaObject(8, (220, 220, 0), carla.CityObjectLabel.TrafficSigns,[

    ])  # traffic sign
    Vegetation = CarlaObject(9, (107, 142, 35), carla.CityObjectLabel.Vegetation)   # vegetation
    Terrain = CarlaObject(10, (152, 251, 152), carla.CityObjectLabel.Terrain)    # terrain
    Sky = CarlaObject(11, (70, 130, 180), carla.CityObjectLabel.Sky)         # sky
    Pedestrians = CarlaObject(12, (220, 20, 60), carla.CityObjectLabel.Pedestrians,[
        "walker.pedestrian.0049",
        "walker.pedestrian.0048",
        "walker.pedestrian.0047",
        "walker.pedestrian.0046",
        "walker.pedestrian.0045",
        "walker.pedestrian.0044",
        "walker.pedestrian.0043",
        "walker.pedestrian.0042",
        "walker.pedestrian.0041",
        "walker.pedestrian.0040",
        "walker.pedestrian.0039",
        "walker.pedestrian.0038",
        "walker.pedestrian.0037",
        "walker.pedestrian.0036",
        "walker.pedestrian.0035",
        "walker.pedestrian.0034",
        "walker.pedestrian.0033",
        "walker.pedestrian.0032",
        "walker.pedestrian.0031",
        "walker.pedestrian.0030",
        "walker.pedestrian.0029",
        "walker.pedestrian.0028",
        "walker.pedestrian.0027",
        "walker.pedestrian.0026",
        "walker.pedestrian.0025",
        "walker.pedestrian.0024",
        "walker.pedestrian.0023",
        "walker.pedestrian.0022",
        "walker.pedestrian.0021",
        "walker.pedestrian.0020",
        "walker.pedestrian.0019",
        "walker.pedestrian.0018",
        "walker.pedestrian.0017",
        "walker.pedestrian.0016",
        "walker.pedestrian.0015",
        "walker.pedestrian.0014",
        "walker.pedestrian.0013",
        "walker.pedestrian.0012",
        "walker.pedestrian.0011",
        "walker.pedestrian.0010",
        "walker.pedestrian.0009",
        "walker.pedestrian.0008",
        "walker.pedestrian.0007",
        "walker.pedestrian.0006",
        "walker.pedestrian.0005",
        "walker.pedestrian.0004",
        "walker.pedestrian.0003",
        "walker.pedestrian.0002",
        "walker.pedestrian.0001",
    ])  # pedestrian
    Rider = CarlaObject(13, (255, 0, 0), carla.CityObjectLabel.Rider)          # rider
    Car = CarlaObject(14, (0, 0, 142), carla.CityObjectLabel.Car,[
        'vehicle.audi.a2',
        'vehicle.audi.etron',
        'vehicle.audi.tt',
        'vehicle.bmw.grandtourer',
        'vehicle.chevrolet.impala',
        'vehicle.citroen.c3',
        'vehicle.dodge.charger_2020',
        'vehicle.dodge.charger_police',
        'vehicle.dodge.charger_police_2020',
        'vehicle.ford.crown',
        'vehicle.ford.mustang',
        'vehicle.jeep.wrangler_rubicon',
        'vehicle.lincoln.mkz_2017',
        'vehicle.lincoln.mkz_2020',
        'vehicle.mercedes.coupe',
        'vehicle.mercedes.coupe_2020',
        'vehicle.micro.microlino',
        'vehicle.mini.cooper_s',
        'vehicle.mini.cooper_s_2021',
        'vehicle.nissan.micra',
        'vehicle.nissan.patrol',
        'vehicle.nissan.patrol_2021',
        'vehicle.seat.leon',
        'vehicle.tesla.model3',
        'vehicle.toyota.prius'
    ])            # car
    Truck = CarlaObject(15, (0, 0, 70), carla.CityObjectLabel.Truck,[
        # Trucks
        'vehicle.carlamotors.carlacola',
        'vehicle.carlamotors.european_hgv',
        'vehicle.carlamotors.firetruck',
        'vehicle.tesla.cybertruck',
        'vehicle.carlamotors.european_hgv',
        'vehicle.tesla.cybertruckvehicle.ford.ambulance',
        # Vans
        'vehicle.ford.ambulance',
        'vehicle.mercedes.sprinter',
        'vehicle.volkswagen.t2',
        'vehicle.volkswagen.t2_2021'
    ])
    Bus = CarlaObject(16, (0, 60, 100), carla.CityObjectLabel.Bus,
                      ['vehicle.mitsubishi.fusorosa'])           # bus
    Train = CarlaObject(17, (0, 80, 100), carla.CityObjectLabel.Train)         # train
    Motorcycle = CarlaObject(18, (0, 0, 230), carla.CityObjectLabel.Motorcycle,[
        'vehicle.harley-davidson.low_rider',
        'vehicle.kawasaki.ninja',
        'vehicle.vespa.zx125',
        'vehicle.yamaha.yzf'])     # motorcycle
    Bicycle = CarlaObject(19, (119, 11, 32), carla.CityObjectLabel.Bicycle,[
        'vehicle.bh.crossbike',
        'vehicle.diamondback.century',
        'vehicle.gazelle.omafiets'
    ])      # bicycle
    Static = CarlaObject(20, (110, 190, 160), carla.CityObjectLabel.Static)     # static
    Dynamic = CarlaObject(21, (170, 120, 50), carla.CityObjectLabel.Dynamic)     # dynamic
    Other = CarlaObject(22, (55, 90, 80), carla.CityObjectLabel.Other)         # other
    Water = CarlaObject(23, (45, 60, 150), carla.CityObjectLabel.Water)        # water
    RoadLines = CarlaObject(24, (157, 234, 50), carla.CityObjectLabel.RoadLines)   # road line
    Ground = CarlaObject(25, (81, 0, 81), carla.CityObjectLabel.Ground)         # ground
    Bridge = CarlaObject(26, (150, 100, 100), carla.CityObjectLabel.Bridge)     # bridge
    RailTrack = CarlaObject(27, (230, 150, 140), carla.CityObjectLabel.RailTrack)  # rail track
    GuardRail = CarlaObject(28, (180, 165, 180), carla.CityObjectLabel.GuardRail)  # guard rail
    NonDrivable = CarlaObject(29, (255, 255, 255), carla.CityObjectLabel.Other,[
        # Custom your blueprint here.
    ])
    @staticmethod
    def get_all_objects():
        """
        Get a list of all CarlaCityObject instances.
        
        Returns
        -------
        list
            A list of all CarlaCityObject instances.
        """
        all_labels = []
        for label in CarlaObjects.__dict__.values():
            if isinstance(label, CarlaObject):
                all_labels.append(label)
        return all_labels


class CarlaObjectCategories:
    TrafficControl: CarlaObjectCategory = CarlaObjectCategory("Traffic", (250, 170, 30), [
        CarlaObjects.TrafficLight,
        CarlaObjects.TrafficSigns,
        CarlaObjects.Poles,
    ])
    Vehicles: CarlaObjectCategory = CarlaObjectCategory("Vehicle", (0, 0, 142), [
        CarlaObjects.Car,
        CarlaObjects.Truck,
        CarlaObjects.Bus,
        CarlaObjects.Train,
        CarlaObjects.Motorcycle,
        CarlaObjects.Bicycle,
    ])
    Pedestrians: CarlaObjectCategory = CarlaObjectCategory("Pedestrian", (220, 20, 60), [
        CarlaObjects.Pedestrians,
        CarlaObjects.Rider,
    ])
    Road: CarlaObjectCategory = CarlaObjectCategory("Road", (128, 64, 128), [
        CarlaObjects.Roads,
        CarlaObjects.Sidewalks,
        CarlaObjects.RoadLines,
        CarlaObjects.Ground,
        CarlaObjects.Bridge,
        CarlaObjects.RailTrack,
        CarlaObjects.GuardRail,
    ])
    Building: CarlaObjectCategory = CarlaObjectCategory("Building", (70, 70, 70), [
        CarlaObjects.Buildings,
        CarlaObjects.Walls,
        CarlaObjects.Fences,
    ])
    Vegetation: CarlaObjectCategory = CarlaObjectCategory("Vegetation", (107, 142, 35), [
        CarlaObjects.Vegetation,
        CarlaObjects.Terrain,
    ])
    NonDrivable: CarlaObjectCategory = CarlaObjectCategory("NonDrivable", (255, 255, 255), [
        CarlaObjects.NonDrivable,
    ])

    @staticmethod
    def get_all_categories()->List[CarlaObjectCategory]:
        all_categories = []
        for category in CarlaObjectCategories.__dict__.values():
            if isinstance(category, CarlaObjectCategory):
                all_categories.append(category)
        return all_categories
    @staticmethod
    def all_blueprint_ids()->List[str]:
        blueprints = []
        all_categories = CarlaObjectCategories.get_all_categories()
        for category in all_categories:
            list_of_object_blueprints = category.get_blueprints()
            for obj, bps in list_of_object_blueprints:
                blueprints.extend(bps)
        return blueprints




class ColorConstants:
    VIRIDIS = np.array(cm.get_cmap('plasma').colors)
    VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
    LABEL_COLORS = np.array([
        CarlaObjects.NoneLabel.color,
        CarlaObjects.Roads.color,
        CarlaObjects.Sidewalks.color,
        CarlaObjects.Buildings.color,
        CarlaObjects.Walls.color,
        CarlaObjects.Fences.color,
        CarlaObjects.Poles.color,
        CarlaObjects.TrafficLight.color,
        CarlaObjects.TrafficSigns.color,
        CarlaObjects.Vegetation.color,
        CarlaObjects.Terrain.color,
        CarlaObjects.Sky.color,
        CarlaObjects.Pedestrians.color,
        CarlaObjects.Rider.color,
        CarlaObjects.Car.color,
        CarlaObjects.Truck.color,
        CarlaObjects.Bus.color,
        CarlaObjects.Train.color,
        CarlaObjects.Motorcycle.color,
        CarlaObjects.Bicycle.color,
        CarlaObjects.Static.color,
        CarlaObjects.Dynamic.color,
        CarlaObjects.Other.color,
        CarlaObjects.Water.color,
        CarlaObjects.RoadLines.color,
        CarlaObjects.Ground.color,
        CarlaObjects.Bridge.color,
        CarlaObjects.RailTrack.color,
        CarlaObjects.GuardRail.color,
        CarlaObjects.NonDrivable.color,
    ]) / 255.0  # normalize each channel [0-1] since that's what Open3D uses
    @staticmethod
    def get_color(label: Union[CarlaObject, CarlaObjectCategory]):
        if isinstance(label, CarlaObject):
            return ColorConstants.LABEL_COLORS[label.id]
        elif isinstance(label, CarlaObjectCategory):
            return label.color
        else:
            raise TypeError(f"Expected label to be of type {CarlaObject} or {CarlaObjectCategory}, but got {type(label)}")
    @staticmethod
    def get_color_from_id(id: int):
        return ColorConstants.LABEL_COLORS[id]
    @staticmethod
    def get_color_from_name(name: str):
        return ColorConstants.get_color(getattr(CarlaObjects, name))


CUSTOM2CARLA = {
    CarlaObjects.NoneLabel: carla.CityObjectLabel.NONE,
    CarlaObjects.Roads: carla.CityObjectLabel.Roads,
    CarlaObjects.Sidewalks: carla.CityObjectLabel.Sidewalks,
    CarlaObjects.Buildings: carla.CityObjectLabel.Buildings,
    CarlaObjects.Walls: carla.CityObjectLabel.Walls,
    CarlaObjects.Fences: carla.CityObjectLabel.Fences,
    CarlaObjects.Poles: carla.CityObjectLabel.Poles,
    CarlaObjects.TrafficLight: carla.CityObjectLabel.TrafficLight,
    CarlaObjects.TrafficSigns: carla.CityObjectLabel.TrafficSigns,
    CarlaObjects.Vegetation: carla.CityObjectLabel.Vegetation,
    CarlaObjects.Terrain: carla.CityObjectLabel.Terrain,
    CarlaObjects.Sky: carla.CityObjectLabel.Sky,
    CarlaObjects.Pedestrians: carla.CityObjectLabel.Pedestrians,
    CarlaObjects.Rider: carla.CityObjectLabel.Rider,
    CarlaObjects.Car: carla.CityObjectLabel.Car,
    CarlaObjects.Truck: carla.CityObjectLabel.Truck,
    CarlaObjects.Bus: carla.CityObjectLabel.Bus,
    CarlaObjects.Train: carla.CityObjectLabel.Train,
    CarlaObjects.Motorcycle: carla.CityObjectLabel.Motorcycle,
    CarlaObjects.Bicycle: carla.CityObjectLabel.Bicycle,
    CarlaObjects.Static: carla.CityObjectLabel.Static,
    CarlaObjects.Dynamic: carla.CityObjectLabel.Dynamic,
    CarlaObjects.Other: carla.CityObjectLabel.Other,
    CarlaObjects.Water: carla.CityObjectLabel.Water,
    CarlaObjects.RoadLines: carla.CityObjectLabel.RoadLines,
    CarlaObjects.Ground: carla.CityObjectLabel.Ground,
    CarlaObjects.Bridge: carla.CityObjectLabel.Bridge,
    CarlaObjects.RailTrack: carla.CityObjectLabel.RailTrack,
    CarlaObjects.GuardRail: carla.CityObjectLabel.GuardRail,
    CarlaObjects.NonDrivable: carla.CityObjectLabel.NONE,
}

CARLA2CUSTOM = {v: k for k, v in CUSTOM2CARLA.items()}
ALL_CARLA_LABELS = list(CARLA2CUSTOM.keys())

def get_actors_from_list(world: carla.World, object_list: List[Union[CarlaObjectCategory,CarlaObjects]]):
    """
    Get a list of actors from a list of CityObjectLabels or CityObjectCategories
    
    Parameters
    ----------
        world : carla.World
            The world to get the actors from.
        object_list : list
            A list of CityObjectLabels or CityObjectCategories.

    Returns
    -------
    list
        A list of actors.
    """
    actors = []
    for obj in object_list:
        if isinstance(obj, CarlaObjectCategory):
            actors.extend(get_actors_from_list(world, obj.children))
        elif isinstance(obj, CarlaObject):
            actors.extend(world.get_environment_objects(obj.carla_type))
        else:
            raise TypeError(f"Expected objects in 'object_list' to be of type {CarlaObject} or {CarlaObjectCategory}, but got {type(obj)}")
    # remove duplicates
    actors = list(set(actors))
    return actors


def get_bounding_boxes(world: carla.World, object_list: List[Union[CarlaObjectCategory,CarlaObjects]]) -> carla.BoundingBox:
    """
    Get the bounding boxes of the vehicles in the world

    Parameters
    ----------
    world : carla.World
        The world to get the bounding boxes from
    object_list : list[CarlaObjectCategory,CarlaObjects]
        The list of objects to get the bounding boxes from

    Returns
    -------
    carla.BoundingBox
        The bounding boxes of the vehicles in the world
    """
    bboxes = []
    for obj in object_list:
        if isinstance(obj, CarlaObjectCategory):
            bboxes.extend(get_bounding_boxes(world, obj.children))
        elif isinstance(obj, CarlaObject):
            bboxes.extend(world.get_level_bbs(obj.carla_type))
        else:
            raise TypeError(f"Expected objects in 'object_list' to be of type {CarlaObject} or {CarlaObjectCategory}, but got {type(obj)}")
    # remove duplicates
    bboxes = list(set(bboxes))
    return bboxes

def blueprint2carla_object(blueprint: carla.ActorBlueprint) -> CarlaObject:
    """
    Convert a carla.ActorBlueprint to a CarlaObject.
    
    Parameters
    ----------
    blueprint : carla.ActorBlueprint
        The blueprint to convert.
    
    Returns
    -------
    CarlaObject
        The CarlaObject corresponding to the blueprint.
    """
    for obj in CarlaObjects.get_all_objects():
        if blueprint.id in obj.blueprint_ids:
            return obj
    raise ValueError(f"Blueprint {blueprint} does not have a corresponding CarlaObject")