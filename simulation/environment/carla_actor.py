from dataclasses import dataclass
from typing import Union

import carla
import numpy as np

from utils.carla_objects import (CarlaObject, CarlaObjectCategories,
                                 CarlaObjects)


@dataclass
class StationaryActor:
    env_object: carla.EnvironmentObject
    location: carla.Location
    def __init__(self, env_object: carla.EnvironmentObject, location: carla.Location):
        assert isinstance(env_object, carla.EnvironmentObject), f"env_object must be of type carla.EnvironmentObject, not {type(env_object)}"
        assert isinstance(location, carla.Location), f"location must be of type carla.Location, not {type(location)}"
        self.env_object = env_object
        self.location = location
        self.bounding_box = env_object.bounding_box

    def get_transform(self)->carla.Transform:
        return carla.Transform(self.location, self.bounding_box.rotation)
    def get_location(self):
        return self.location
    def get_bounding_box(self):
        return self.bounding_box
    def rotation_matrix(self):
        return self.get_transform().get_matrix()[0:3,0:3]
    @property
    def rotation(self)->carla.Rotation:
        """
        Get the rotation of the actor
        
        Returns
        -------
        carla.Rotation
            The rotation of the actor
        """
        return self.bounding_box.rotation
    @property
    def velocity(self)->carla.Vector3D:
        """
        Get the velocity of the actor
        
        Returns
        -------
        carla.Vector3D
            The velocity of the actor
        """
        return carla.Vector3D((0,0,0))
    @property
    def acceleration(self)->carla.Vector3D:
        """
        Get the acceleration of the actor
        
        Returns
        -------
        carla.Vector3D
            The acceleration of the actor
        """
        return carla.Vector3D((0,0,0))
    @property
    def id(self):
        return self.env_object.id
    def destroy(self):
        pass

class CarlaActor:
    """
    Wrapper of the carla.Actor class with more consistent object labeling.
    
    Parameters
    ----------
    actor : carla.Actor
        The Carla actor
    label : CarlaObject
        The CarlaObject is a description of the type of actor including its color, id and type.
        
    Attributes
    ----------
    actor : carla.Actor or carla.EnvironmentObject
        The Carla actor
    label : CarlaObject
        The CarlaObject is a description of the type of actor including its color, id and type.
    bounding_box : carla.BoundingBox
        The bounding box of the actor
    location : carla.Location
        The location of the actor
    rotation : carla.Rotation
        The rotation of the actor
    velocity : carla.Vector3D
        The velocity of the actor
    acceleration : carla.Vector3D
        The acceleration of the actor

    Raises
    ------
    ValueError
        If actor is not of type carla.Actor or carla.EnvironmentObject

    """
    def __init__(self, actor: Union[carla.Actor,carla.EnvironmentObject], label: CarlaObject):
        """
        Parameters
        ----------
        actor : carla.Actor
            The Carla actor
        label : CarlaObject
            The CarlaObject is a description of the type of actor including its color, id and type.
        """
        assert isinstance(label, CarlaObject), f"label must be of type CarlaObject, not {type(label)}"
        self._bounding_box = actor.bounding_box
        if isinstance(actor, carla.EnvironmentObject):
            self.actor = StationaryActor(actor, actor.bounding_box.location)
        elif isinstance(actor, carla.Actor):
            self.actor = actor
        else:
            raise ValueError(f"actor must be of type carla.Actor or carla.EnvironmentObject, not {type(actor)}")
        self.label = label
    def get_transform(self)->carla.Transform:
        """
        Get the transform of the actor
        
        Returns
        -------
        carla.Transform
            The transform of the actor
        """
        return self.actor.get_transform()
    def get_location(self):
        """
        Get the location of the actor
        
        Returns
        -------
        carla.Location
            The location of the actor
        """
        return self.location
    def get_velocity(self)->carla.Vector3D:
        """
        Get the velocity of the actor
        
        Returns
        -------
        carla.Vector3D
            The velocity of the actor
        """
        return self.actor.get_velocity()
    def get_acceleration(self)->carla.Vector3D:
        """
        Get the acceleration of the actor
        
        Returns
        -------
        carla.Vector3D
            The acceleration of the actor
        """
        return self.actor.get_acceleration()
    def rotation_matrix(self):
        """
        Get the rotation matrix of the actor
        
        Returns
        -------
        np.ndarray
            The rotation matrix of the actor
        """
        return np.array(self.actor.get_transform().get_matrix())[0:3,0:3]


        
    @property
    def bounding_box(self)->carla.BoundingBox:
        """
        Get the bounding box of the actor
        
        Returns
        -------
        carla.BoundingBox
            The bounding box of the actor
        """
        self._bounding_box.location = self.location
        if not isinstance(self.actor, StationaryActor):
            self._bounding_box.location.z += self._bounding_box.extent.z
        self._bounding_box.rotation = self.rotation
        return self._bounding_box
    
    @property
    def location(self)->carla.Location:
        """
        Get the location of the actor
        
        Returns
        -------
        carla.Location
            The location of the actor
        """
        return self.actor.get_location()
    
    @property
    def rotation(self)->carla.Rotation:
        """
        Get the rotation of the actor
        
        Returns
        -------
        carla.Rotation
            The rotation of the actor
        """
        return self.actor.get_transform().rotation
    @property
    def transform(self)->carla.Transform:
        """
        Get the transform of the actor
        
        Returns
        -------
        carla.Transform
            The transform of the actor
        """
        return self.actor.get_transform()
    @property
    def velocity(self)->carla.Vector3D:
        """
        Get the velocity of the actor
        
        Returns
        -------
        carla.Vector3D
            The velocity of the actor
        """
        return self.actor.get_velocity()
    @property
    def acceleration(self)->carla.Vector3D:
        """
        Get the acceleration of the actor
        
        Returns
        -------
        carla.Vector3D
            The acceleration of the actor
        """
        return self.actor.get_acceleration()
    def destroy(self):
        """
        Destroy the actor
        """
        self.actor.destroy()
    
    def __eq__(self, other):
        return self.actor.id == other.actor.id
    def __hash__(self):
        return hash(self.actor.id)
    def __str__(self):
        return f"CarlaActor(id={self.actor.id}, label={self.label})"
    
    
    
        
