import time
import numpy as np
import carla
class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()
    

def deg2rad(deg):
    return deg * np.pi / 180.0
def rad2deg(rad):
    return rad * 180.0 / np.pi

def to_list(x,obj:object):
    if x is None:
        return []
    if not isinstance(x, list):
        x = [x]
    assert all(issubclass(type(i), obj) for i in x), f"Expected all elements of x to be of type {obj}, but got {[type(i) for i in x]}"
    return x

def carla_vec2np_array(vec:carla.Vector3D):
    return np.array([vec.x, vec.y, vec.z])


def reset_world(client:carla.Client):
    world = client.get_world()
    original_settings = world.get_settings()
    actor_list = world.get_actors()
    for actor in actor_list:
        if actor.type_id.startswith("vehicle") or actor.type_id.startswith("sensor") or actor.type_id.startswith("traffic") or actor.type_id.startswith("controller"):
            actor.destroy()
    print("All vehicles destroyed")
    world.apply_settings(original_settings)
    print("World reset")
    return world

def destroy_all_vehicles(world:carla.World):
    actor_list = world.get_actors()
    vehicle_blueprints = world.get_blueprint_library().filter("vehicle.*")
    types = [x.id for x in vehicle_blueprints]
    destroyed = 0
    for actor in actor_list:
        if actor.type_id in types:
            actor.destroy()
            destroyed += 1
    print(f"Destroyed {destroyed} vehicles")

from queue import LifoQueue
from threading import Lock

class ThreadSafeStack:
    def __init__(self, maxsize=1):
        self.stack = LifoQueue(maxsize=maxsize)
        self.is_empty = True

    def put(self, item, block:bool=True, timeout:float=None):
        if self.is_empty:
            self.stack.put(item, block=block, timeout=timeout)
            self.is_empty = False
        else:
            # Empty the stack
            while not self.stack.empty():
                self.stack.get()
            self.stack.put(item, block=block, timeout=timeout)

    def get(self, block:bool=True, timeout:float=None):
        if not self.is_empty:
            item = self.stack.get(block=block, timeout=timeout)
            self.is_empty = True
            return item
        else:
            raise IndexError("Stack is empty.")