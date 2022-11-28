
from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.container import ObstacleContainer


class OptitrackContainer(ObstacleContainer):
    def append(self, obstacle: Obstacle):
        # TODO: create new topic each time
        if not hasattr(obstacle, "name"):
            raise ValueError("Topic name 'name' needed to recognize obstacle")

    def callback(self):
        pass
