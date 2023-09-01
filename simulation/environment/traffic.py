import random
import carla
import time
class CarlaTrafficSpawner:
    def __init__(self, client:carla.Client, world: carla.World):
        self.client = client
        self.world = world
        self.blueprint_library = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.actors = []

    def spawn_traffic(self, num_vehicles=10, num_pedestrians=10):
        blueprints = self.blueprint_library.filter('vehicle.*')
        vehicles = []
        pedestrians = []
        for _ in range(num_pedestrians):
            spawn_point = random.choice(self.spawn_points)
            vehicle_bp = random.choice(blueprints)
            try:
                actor = self.world.spawn_actor(vehicle_bp, spawn_point)
            except:
                continue
            vehicles.append(actor)
        print(f"Spawned {len(vehicles)} vehicles")
        
        
        blueprints = self.blueprint_library.filter('walker.*')
        for _ in range(num_pedestrians):
            spawn_point = self.world.get_random_location_from_navigation()
            vehicle_bp = random.choice(blueprints)
            try:
                actor = self.world.spawn_actor(vehicle_bp, spawn_point)
            except:
                continue
            pedestrians.append(actor)
        print(f"Spawned {len(pedestrians)} pedestrians")
        
        
        self.actors = vehicles + pedestrians
        
        
        
        
        
        
        
    def start_traffic(self):
        for actor in self.actors:
            if actor.type_id.startswith("vehicle"):
                actor.set_autopilot(True)
            elif actor.type_id.startswith("walker"):
                actor.set_simulate_physics(True)
                actor.apply_control(carla.WalkerControl())

    def destroy_traffic(self):
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actors])

# Example usage
if __name__ == '__main__':
    from pynput.keyboard import Key, Listener
    def destroy_all_vehicle(world: carla.World):
        actor_list = world.get_actors()
        for actor in actor_list:
            if actor.type_id.startswith("vehicle"):
                actor.destroy()
        print("All vehicles destroyed")
    def on_press(key):
        print(key)
    def on_release(key):
        print(key)
        if key == Key.esc:
            return False
    host = 'localhost'
    port = 2000
    client = carla.Client(host, port)
    client.set_timeout(2.0)

    world = client.get_world()
    original_settings = world.get_settings()
    destroy_all_vehicle(world)
    spawner = CarlaTrafficSpawner(client, world)
    spawner.spawn_traffic(num_vehicles=50)
    spawner.start_traffic()

    # Run the main loop or perform other tasks
    with Listener(on_press=on_press, on_release=on_release) as listener:
        while True:
            world.tick()
            time.sleep(0.1)
            q = listener.join()
            if not q:
                break
    try:
        client.apply_batch([carla.command.DestroyActor(x) for x in spawner.actors])
        # Restore the original settings of the world
        world.apply_settings(original_settings)
    except:
        pass
    # spawner.destroy_traffic()

    # Destroy all actors