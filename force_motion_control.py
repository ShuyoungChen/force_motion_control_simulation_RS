import numpy as np
import rpi_abb_irc5
import time
import general_robotics_toolbox as rox
import QuadProg as qp

def abb_irb6640_180_255_robot():
    """Return a Robot instance for the ABB IRB6640 180-255 robot"""
    
    x = np.array([1,0,0])
    y = np.array([0,1,0])
    z = np.array([0,0,1])
    a = np.array([0,0,0])
    
    H = np.array([z,y,y,x,y,x]).T
    P = np.array([0.78*z, 0.32*x, 1.075*z, 0.2*z, 1.142*x, 0.2*x, a]).T
    joint_type = [0,0,0,0,0,0]
    joint_min = np.deg2rad(np.array([-170, -65, -180, -300, -120, -360]))
    joint_max = np.deg2rad(np.array([170, 85, 70, 300, 120, 360]))
    
    p_tool = np.array([0,0,0])
    R_tool = rox.rot([0,1,0], np.pi/2.0)
    
    return rox.Robot(H, P, joint_type, joint_min, joint_max, R_tool=R_tool, p_tool=p_tool) 
	
def main():    
    # initialize EGM interface instance
    egm = rpi_abb_irc5.EGM()
    
    # initialize a robot instance
    abb_robot = abb_irb6640_180_255_robot()

    # desired force
    Fd = -1000
	
    # desired velocity in x
    vdx = 1
	
    # spring constant
    k = 50000
	
    # position of object in z
    pos_obj = 1.5
	
    # feedback gain
    Kp = 0.005
    Kd = 0.2
    Ki = 0.001
	
    # time step
    delta_t = 0.004*4
	
    # initial configuration in degree
    init = [0,0,0,0,90,0]

    # quadprog to solve for joint velocity
    quadprog = qp.QuadProg(abb_robot)
	
    # determine if robot has reached the initial configuration init
    tag = True
    while tag:
        res, state = egm.receive_from_robot(.1)
        if res: 
            #print np.fabs(sum(state.joint_angles) - sum(b))
            if np.fabs(sum(state.joint_angles) - sum(init)) < 1e-2:
                tag = False
	
    time.sleep(0.5)	
    print '--------start force control--------'
	
    while True:
        # receive EGM feedback
        res, state = egm.receive_from_robot(.1)
        
        if not res:
            continue
       
	    # forward kinematics to calculate current position of eef
        pose = rox.fwdkin(abb_robot, np.deg2rad(state.joint_angles))

        if pose.p[2] >= pos_obj:
            F = 0
            v_l = np.array([0, 0, -Kp*(F-Fd)])
        else:
            F = k*(pose.p[2] - pos_obj)
            v_l = np.array([vdx, 0, -Kp*(F-Fd)])
			
        print F
		
        # formalize entire twist
        spatial_velocity_command = np.array([0, 0, 0, v_l[0], v_l[1], v_l[2]])

        # solve for joint velocity
        # Jacobian inverse
        #J = rox.robotjacobian(abb_robot, np.deg2rad(state.joint_angles))		
        #joints_vel = np.linalg.pinv(J).dot(spatial_velocity_command)
		
        # QP
        joints_vel = quadprog.compute_joint_vel_cmd_qp(np.deg2rad(state.joint_angles), spatial_velocity_command)
		
        # commanded joint position setpoint to EGM
        q_c = np.deg2rad(state.joint_angles) + joints_vel*delta_t
		
        egm.send_to_robot(q_c)
        	
if __name__ == '__main__':
    main()