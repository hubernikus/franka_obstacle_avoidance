from dynamic_obstacle_avoidance.obstacles import Obstacle
from dynamic_obstacle_avoidance.container import ObstacleContainer


class OptitrackContainer(ObstacleContainer):
    def __init__(self):
        obstacle_ids = []
        obstacle_offsets = []

        # Setup full optitrack callback
        
    def append(
        self,
        obstacle: Obstacle,
        obstacle_id: int,
        start_position: 
    ):
        super().append(obstacle)
        # TODO: create new topic each time
        if not hasattr(obstacle, "name"):
            raise ValueError("Topic name 'name' needed to recognize obstacle")

        pass
    
    def callback(self):
        pass
