from typing import Tuple, List
import threading
import pygame
import time
from simulation.sensors.sensor import Sensor


class DisplayManager:
    """
    A class for managing and displaying multiple sensors in a grid layout.

    Parameters
    ----------
    grid_size : Tuple[int, int]
        The size of the grid in rows and columns.
    window_size : Tuple[int, int]
        The size of the display window.
    display_index : int, optional
        The index of the display to use. Defaults to 0.
    """

    def __init__(self, grid_size: Tuple[int, int], window_size: Tuple[int, int], display_index: int = 0):
        self.grid_size = grid_size
        self.window_size = window_size
        self.window_data_list: List[Tuple[Sensor, Tuple[int, int]]] = []
        self.added = 0
        self.window_title = "Display Manager"
        self.running = False
        self.lock = threading.Lock()
        self.t0 = time.time()

    def get_window_size(self) -> List[int]:
        """
        Get the size of the display window.

        Returns
        -------
        List[int]
            The size of the window as a list [width, height].
        """
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self) -> List[int]:
        """
        Get the size of each display in the grid.

        Returns
        -------
        List[int]
            The size of each display as a list [width, height].
        """
        return [int(self.window_size[0] / self.grid_size[1]), int(self.window_size[1] / self.grid_size[0])]

    def get_display_offset(self, gridPos: Tuple[int, int]) -> List[int]:
        """
        Get the offset position for a display in the grid.

        Parameters
        ----------
        gridPos : Tuple[int, int]
            The position of the display in the grid.

        Returns
        -------
        List[int]
            The offset position of the display as a list [x, y].
        """
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor: Sensor, position: Tuple[int, int] = None):
        """
        Add a sensor to the display manager.

        Parameters
        ----------
        sensor : Sensor
            The sensor to add.
        position : Tuple[int, int], optional
            The position of the sensor in the grid. Defaults to None.
        """
        assert isinstance(sensor, Sensor), f"Expected sensor to be of type 'Sensor', but got {type(sensor)}"
        assert sensor.is_2d, f"Expected sensor to be 2D, but got 3D"
        if position is None:
            row = self.added // self.grid_size[1]
            col = self.added % self.grid_size[1]
            position = (row, col)
        position = tuple(position) if not isinstance(position, tuple) else position
        self.window_data_list.append((sensor, position))

    def get_sensor_list(self) -> List[Sensor]:
        """
        Get the list of sensors added to the display manager.

        Returns
        -------
        List[Sensor]
            The list of sensors.
        """
        return [sensor for sensor, _ in self.window_data_list]

    def start(self):
        """
        Start the display manager and render the sensors.
        """
        self.running = True
        self.thread = threading.Thread(target=self._update_visualizer)
        self.thread.start()

    def stop(self):
        """
        Stop the display manager.
        """
        # Wait for the thread to finish
        self.running = False
        self.thread.join()

    def _update_visualizer(self):
        """
        Update the visualizer and handle events in a separate thread.
        """
        pygame.init()
        self.display = pygame.display.set_mode(self.window_size)
        pygame.display.set_caption(self.window_title)
        while self.running:
            with self.lock:
                self.render()
        pygame.quit()

    def render(self):
        """
        Render the sensors on the display.
        """
        for sensor, position in self.window_data_list:
            scene = sensor.get_scene()
            if scene is not None:
                offset = self.get_display_offset(position)
                scene = pygame.surfarray.make_surface(scene.swapaxes(0, 1))
                self.display.blit(scene, offset)
        pygame.display.flip()

    def destroy(self):
        """
        Destroy the display manager and associated sensors.
        """
        for s, _ in self.window_data_list:
            s.destroy()

    def check_events(self):
        """
        Check for events and handle them.
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                break
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    self.running = False
                    break

    def is_running(self) -> bool:
        """
        Check if the display manager is running.

        Returns
        -------
        bool
            True if running, False otherwise.
        """
        return self.running





