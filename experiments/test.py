class Drone:
    def __init__(self):
        self.u = [0, 0, 0]

class Controller:
    def __init__(self, drone):
        self.drone = drone
    
    def update(self):
        self.drone.u[1] = 5

drone = Drone()
controller = Controller(drone)

controller.update()
print(drone.u)  # Output: [0, 5, 0]
