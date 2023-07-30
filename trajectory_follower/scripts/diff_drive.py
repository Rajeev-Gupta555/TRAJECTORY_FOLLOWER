import math, time

class Diff_drive():
    def __init__(self, wheel_radii, wheel_separation):
        self.wheel_radii = wheel_radii
        self.wheel_separation = wheel_separation
        self.posn = [0.0, 0.0, 0.0]
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.max_linear_vel = 0.15          #m/s
        self.max_angular_vel = 1            #rad/s
        self.forward_vel = self.max_linear_vel
        self.angular_vel = self.max_angular_vel
    
    def update_odom(self, t_step):
        vl = self.cmd_vel[0] - self.cmd_vel[2] * self.wheel_separation / 2.0
        vr = self.cmd_vel[0] + self.cmd_vel[2] * self.wheel_separation / 2.0
        sl = vl * t_step
        sr = vr * t_step
        ssum = sl + sr
        sdiff = sr - sl
        dx = ( ssum ) /2.0 * math.cos ( self.posn[2] + ( sdiff ) / ( 2.0*self.wheel_separation ) )
        dy = ( ssum ) /2.0 * math.sin ( self.posn[2] + ( sdiff ) / ( 2.0*self.wheel_separation ) )
        dtheta = ( sdiff ) / self.wheel_separation
        self.posn[0] += dx
        self.posn[1] += dy
        self.posn[2] += dtheta
        
    def set_forwaard_vel(self, vel):
        self.forward_vel = vel
        
    def move(self, cmd_vector):
        xi = cmd_vector[0]
        yi = cmd_vector[1]
        th = math.atan2(yi, xi) 
        alpha = th
        err_th1 = (alpha-self.posn[2])
        self.cmd_vel[0] = self.forward_vel * 2/(1+math.e**abs(err_th1))
        self.cmd_vel[2] = err_th1 * self.angular_vel
        # print(alpha, err_th1)
        return self.cmd_vel
    
    
if __name__=='__main__':
    mini_bot = Diff_drive(0.04, 0.10)
    target = [-5, 5]
    dist = ((mini_bot.posn[0]-target[0])**2 + (mini_bot.posn[1]-target[1])**2)**0.5
    t_step = 0.010
    while(dist>0.001):
        print(mini_bot.posn)
        vector = [-(mini_bot.posn[0]-target[0])/dist, -(mini_bot.posn[1]-target[1])/dist]
        mini_bot.move(vector)
        time.sleep(t_step)
        dist = ((mini_bot.posn[0]-target[0])**2 + (mini_bot.posn[1]-target[1])**2)**0.5
        mini_bot.update_odom(t_step)