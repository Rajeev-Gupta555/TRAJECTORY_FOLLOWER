import math, time

class Holonomic_drive():
    def __init__(self, wheel_radii, wheel_separation):
        self.wheel_radii = wheel_radii
        self.wheel_separation = wheel_separation
        self.posn = [0.0, 0.0, 0.0]
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.forward_vel = 0.15         #m/s
        self.angular_vel = 1.00         #rad/s
    
    def update_odom(self, t_step):
        # vl = self.cmd_vel[0] - self.cmd_vel[2] * self.wheel_separation / 2.0
        # vr = self.cmd_vel[0] + self.cmd_vel[2] * self.wheel_separation / 2.0
        # sl = vl * t_step
        # sr = vr * t_step
        # ssum = sl + sr
        # sdiff = sr - sl
        # dx = ( ssum ) /2.0 * math.cos ( self.posn[2] + ( sdiff ) / ( 2.0*self.wheel_separation ) )
        # dy = ( ssum ) /2.0 * math.sin ( self.posn[2] + ( sdiff ) / ( 2.0*self.wheel_separation ) )
        # dtheta = ( sdiff ) / self.wheel_separation
        dx = self.cmd_vel[0]*t_step
        dy = self.cmd_vel[1]*t_step
        dtheta = 0
        self.posn[0] += dx
        self.posn[1] += dy
        self.posn[2] += dtheta
        # self.posn[2] += 2*math.pi if self.posn[2] < 0 else 0
        # self.posn[2] -= 2*math.pi if self.posn[2] > 2*math.pi else 0
        
    def set_forwaard_vel(self, vel):
        self.forward_vel = vel
        
    def move(self, cmd_vector):
        mod_vector = (cmd_vector[0]**2 + cmd_vector[1]**2)**0.5
        self.cmd_vel[0] = cmd_vector[0]/mod_vector * self.forward_vel
        self.cmd_vel[1] = cmd_vector[1]/mod_vector * self.forward_vel
        return self.cmd_vel
    
    
if __name__=='__main__':
    mini_bot = Holonomic_drive(0.04, 0.10)
    target = [-1, 2]
    dist = ((mini_bot.posn[0]-target[0])**2 + (mini_bot.posn[1]-target[1])**2)**0.5
    t_step = 0.010
    while(dist>0.001):
        print(mini_bot.posn)
        vector = [-(mini_bot.posn[0]-target[0])/dist, -(mini_bot.posn[1]-target[1])/dist]
        mini_bot.move(vector)
        time.sleep(t_step)
        dist = ((mini_bot.posn[0]-target[0])**2 + (mini_bot.posn[1]-target[1])**2)**0.5
        mini_bot.update_odom(t_step)