

class Statemachine():
    def idle(self, rexarm):
        #state that runs by default
        rexarm.torque_multiplier = 0/100.0
        rexarm.cmd_publish()
        
    def moving(self, rexarm, pos, speed):
        rexarm.speed = speed
        rexarm.torque_multiplier = 1
        self.max_torque = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0] 
        rexarm.speed_multiplier = 1
        for i in range (rexarm.num_joints):
            if pos[i] != -200:
                rexarm.joint_angles[i] = pos[i]
        # rexarm.joint_angles = pos
        rexarm.cmd_publish()

    def grapping(self, rexarm):
        rexarm.speed = speed
        rexarm.torque_multiplier = 1
        rexarm.speed_multiplier = 1
        rexarm.joint_angles = pos
        rexarm.cmd_publish()

    def estop(self, rexarm):
        #sets the torque of all motors to 0, in case of emergency
        rexarm.max_torque = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rexarm.cmd_publish()
        
    
