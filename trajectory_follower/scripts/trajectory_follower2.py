import math
import matplotlib.pyplot as plt
import time 
from holonomic_drive import Holonomic_drive
import numpy as np

class Path_Planner():
    def __init__(self, x_min, y_min, x_max, y_max, function=''):
        self.function = function
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.max_iter = 1000
        self.step_size = 0.1
        self.x_init = self.x_min
        self.x_closest = 1e10
        self.y_closest = 1e10
        self.o_closest = 1e10           #theta
        self.x_goal = 1e10
        self.y_goal = 1e10
        self.dist_tol = 0.01
        self.t_step = 0.010
        self.drive = Holonomic_drive(0.0, 0.0)
    
    def set_x_init(self, x_init):
        self.x_init = x_init
        
    def f(self, x):
        return eval(compile(self.function, "<string>", "eval"))
    
    def get_distance(self, x1, y1, x2, y2):
        return ((x1-x2)**2+(y1-y2)**2)**0.5
    
    def get_gradient(self, x, y, x0, y0):
        try:
            return ((x-x0) + 2*x*(self.f(x)-y0))/(((x-x0)**2+(self.f(x)-y0)**2)**0.5)
        except ZeroDivisionError:
            return 0
        
    def get_closest_point(self, x0, y0):
        x_final = x0 if self.x_closest == 1e10 else self.x_closest
        dist = self.dist_tol
        t = 0
        osc = 0
        last_track = [0 for i in range(3)]
        while(osc < 10 and t < self.max_iter):
            t+=1
            x_final -= self.step_size * self.get_gradient(x_final, self.f(x_final), x0, y0)
            dist = self.get_distance(x0, y0, x_final, self.f(x_final))
            last_track[0] = last_track[1]
            last_track[1] = last_track[2]
            last_track[2] = x_final
            if t>100:
                if last_track[0]>0 and last_track[2]>0 and last_track[1]<0 or \
                   last_track[0]<0 and last_track[2]<0 and last_track[1]>0:
                    osc += 1
        self.x_closest = x_final
        self.y_closest = self.f(x_final)
        # print(x_final, self.f(x_final), t)
        return x_final, self.f(x_final)
    
    def get_normal_vector(self):
        x0, y0 = self.drive.posn[0], self.drive.posn[1]
        x1, y1 = self.x_closest, self.y_closest
        mod_vector = ((x1-x0)**2 + (y1-y0)**2)**0.5
        # print([(x1-x0)/mod_vector, (y1-y0)/mod_vector])
        try:
            return [(x1-x0)/mod_vector, (y1-y0)/mod_vector]
        except ZeroDivisionError:
            return [0.0, 0.0]
    
    def get_tanget_vector(self):
        x0, y0 = self.drive.posn[0], self.drive.posn[1]
        # x1, y1 = self.x_closest, self.y_closest
        # mod_vector = ((x1-x0)**2 + (y1-y0)**2)**0.5
        # # print([-(y1-y0)/mod_vector, (x1-x0)/mod_vector])
        # if mod_vector > self.dist_tol:
        #     cmd = [-(y1-y0)/mod_vector, (x1-x0)/mod_vector] 
        #     _cmd = [-a for a in cmd]
        #     if cmd[0]>0 and self.x_goal-x0 > 0 or cmd[0]<0 and self.x_goal-x0 < 0:
        #         return cmd 
        #     else: return _cmd
        x2 = x0 + self.dist_tol if self.x_goal-x0 > 0 \
                                            else x0 - self.dist_tol
        y2 = self.f(x2)
        try:
            mod_vector = ((x2-x0)**2 + (y2-y0)**2)**0.5
            return [(x2-x0)/mod_vector, (y2-y0)/mod_vector]
        except ZeroDivisionError:
            return [0.0, 0.0]
        
        
    def get_cmd(self):
        odom = self.drive.posn
        _x_closest, _y_closest = self.get_closest_point(odom[0], odom[1])
        _dist = self.get_distance(odom[0], odom[1], _x_closest, _y_closest)
        normal_vector = self.get_normal_vector()
        tangent_vector = self.get_tanget_vector()
        mf = 1/(1+(math.e)**(-30*(_dist-0.20)))
        normal_vector[0], normal_vector[1] = normal_vector[0]*mf, normal_vector[1]*mf
        tangent_vector[0], tangent_vector[1] = tangent_vector[0]*(1-mf), tangent_vector[1]*(1-mf)
        print(mf, tangent_vector)
        return [normal_vector[0] + tangent_vector[0], normal_vector[1] + tangent_vector[1]]
        # return [tangent_vector[0], tangent_vector[1]]
    
    def move_to(self, x_goal, y_goal):
        self.x_goal = x_goal
        self.y_goal = y_goal
        x0, y0 = self.drive.posn[0], self.drive.posn[1]
        m = (y_goal-y0)/(x_goal-x0) if x_goal!=x0 else 1e100
        c = y0 - m * x0
        self.function = f"{m}*x + {c}"
        self.follow()
        
    def follow(self):
        plt.figure()
        plt.xlim(self.x_min, self.x_max)
        plt.ylim(self.y_min, self.y_max)
        dist = self.get_distance(self.drive.posn[0], self.drive.posn[1], self.x_goal, self.y_goal)
        try:
            while(dist>self.dist_tol):
                # print(self.drive.posn)
                self.drive.move(self.get_cmd())
                time.sleep(self.t_step)
                self.drive.update_odom(self.t_step)
                plt.plot(self.drive.posn[0], self.drive.posn[1], 'bo', ms=1)
                dist = self.get_distance(self.drive.posn[0], self.drive.posn[1],
                                         self.x_goal, self.y_goal)
        except KeyboardInterrupt:
            print("Destination arrived: Please ensure the safety of your belongings by your own.")
            delta = self.step_size
            x_range = np.arange(self.x_min, self.x_max, delta)
            y_range = self.f(x_range)
            plt.plot(x_range, y_range, 'r-')
            plt.plot(np.zeros(int((self.x_max-self.x_min)/delta)), x_range, 'k-')
            plt.plot(x_range, np.zeros(int((self.x_max-self.x_min)/delta)), 'k-')
            plt.show()
        
    def set_vehicle(self, vehicle):
        self.drive = vehicle
        
        
if __name__ == '__main__':
    mini_bot = Holonomic_drive(0.04, 0.10)
    navigator = Path_Planner(-0.1, -0.1, 1, 1, 'x**2 + 0.2')
    navigator.set_vehicle(mini_bot)
    # navigator.move_to(3, 5)
    navigator.follow()