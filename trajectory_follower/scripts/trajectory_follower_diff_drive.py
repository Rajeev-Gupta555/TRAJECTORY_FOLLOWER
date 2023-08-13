from path_planner_class import Path_Planner
from diff_drive import Diff_drive
        
if __name__ == '__main__':
    mini_bot = Diff_drive(0.04, 0.10)
    navigator = Path_Planner(-0.1, -0.1, 2, 3)
    navigator.set_vehicle(mini_bot)
    # To move the bot to a point---->
    # navigator.move_to(0.5, 0.5)
    
    # To follow a trajectory ------->
    navigator.function = "math.sin(x)"
    navigator.follow()