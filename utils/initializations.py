# Helpful initialization function and classes.
# These include classes for different types of arguments, and functions for initializing different types of systems.
import argparse
import glob
import os
import sys
from typing import Any, Dict, List, Optional, Tuple, Union
import carla
from utils.constants import PathConstants
from utils.process_utils import ProcessResult


def set_carla_path() -> ProcessResult:
    # Set the path to the CARLA simulator
    try:
        sys.path.append(PathConstants.CARLA_PATH)
    except Exception as e:
        print(f"Error setting CARLA path: \n{e}")
        sys.exit(1)
    finally:
        pr = ProcessResult(True, message=f"CARLA path set to {PathConstants.CARLA_PATH}")
        print(pr)
        return pr
    

def warm_up(world:carla.World,ticks:int=20):
    """
    Warm up the simulation by ticking the world a few times.
    
    Parameters
    ----------
    world : carla.World
        The world to tick
    ticks : int, optional
        The number of ticks to warm up for, by default 20
    """
    try:
        for _ in range(ticks):
            world.tick()
    except Exception as e:
        return ProcessResult(False, message=f"Error warming up the simulation: \n{e}")
    else:
        return ProcessResult(True, message=f"Warmed up the simulation for {ticks} ticks")



