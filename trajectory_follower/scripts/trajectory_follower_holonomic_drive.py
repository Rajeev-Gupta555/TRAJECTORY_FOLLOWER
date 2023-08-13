from path_planner_class import Path_Planner
from holonomic_drive import Holonomic_drive       
        
if __name__ == '__main__':
    mini_bot = Holonomic_drive(0.04, 0.10)
    navigator = Path_Planner(-0.1, -0.1, 1, 1, 'x**2 + 0.2')
    navigator.set_vehicle(mini_bot)
    # navigator.move_to(3, 5)
    navigator.follow()