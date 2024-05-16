import time
import numpy as np
from utils import *

import sim
import sys
import modern_robotics as mr
from RRT import RRT

clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5)
if(clientID!=-1):
    print('Connected Successfully')
else: 
    sys.exit('Failed to connect')

returnCode = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
if returnCode == sim.simx_return_ok:
    print('Simulation started')
else:
    print('Failed to start simulation')



class Env:
    def __init__(self, path) -> None:
        self.model_handles = {}
        self.path = path

    def generate_object(self, model, position, orientation = None, inflation = 0.0):
        print(f'Spawning {model} at {position}, {orientation if orientation is not None else [0, 0, 0]}')
        model_path = self.path + model
        
        res, model_handle = sim.simxLoadModel(clientID, model_path, 0, sim.simx_opmode_oneshot_wait)
        if not res == sim.simx_return_ok:
            print('Model loading Failed')
            return
        res = sim.simxSetObjectPosition(clientID, model_handle, -1, position, sim.simx_opmode_oneshot_wait)
        if not res == sim.simx_return_ok:
            print('Pose set Failed')
            return
        if orientation is not None:
            res = sim.simxSetObjectOrientation(clientID, model_handle, -1, orientation, sim.simx_opmode_oneshot_wait)
            if not res == sim.simx_return_ok:
                print('Orientation set Failed')
                return
            
        res, min_x = sim.simxGetObjectFloatParameter(clientID, model_handle, sim.sim_objfloatparam_modelbbox_min_x, sim.simx_opmode_oneshot_wait)
        res, min_y = sim.simxGetObjectFloatParameter(clientID, model_handle, sim.sim_objfloatparam_modelbbox_min_y, sim.simx_opmode_oneshot_wait)
        res, min_z = sim.simxGetObjectFloatParameter(clientID, model_handle, sim.sim_objfloatparam_modelbbox_min_z, sim.simx_opmode_oneshot_wait)
        res, max_x = sim.simxGetObjectFloatParameter(clientID, model_handle, sim.sim_objfloatparam_modelbbox_max_x, sim.simx_opmode_oneshot_wait)
        res, max_y = sim.simxGetObjectFloatParameter(clientID, model_handle, sim.sim_objfloatparam_modelbbox_max_y, sim.simx_opmode_oneshot_wait)
        res, max_z = sim.simxGetObjectFloatParameter(clientID, model_handle, sim.sim_objfloatparam_modelbbox_max_z, sim.simx_opmode_oneshot_wait)
        
        if res == sim.simx_return_ok:
            self.model_handles[model_handle] = {'position':position, 'orientation': orientation, 'bbox':[min_x - inflation, min_y - inflation, min_z + inflation, max_x + inflation, max_y + inflation, max_z + inflation]}
        else:
            print('BBox fetch failed')
            return

        return model_handle

    def plot_object(self, model_handle, position = None, orientation = None, style = 'b-'):
        x,y,z = self.model_handles[model_handle]['position'] if position is None else position
        _, _, yaw = self.model_handles[model_handle]['orientation'] if orientation is None else orientation
        bbox = self.model_handles[model_handle]['bbox']
        plot_rectangle(x, y, bbox, yaw, style)

    def show_env(self,):
        for model_handle in self.model_handles:
            self.plot_object(model_handle)

    def pause_env(self):
        returnCode = sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
        if returnCode == sim.simx_return_ok:
            print('Simulation Paused')
        else:
            print('Failed to pause simulation')

class Bot:
    def __init__(self) -> None:
        error_code, self.youb = sim.simxGetObjectHandle(clientID, '/youBot',sim.simx_opmode_oneshot_wait)
        error_code, self.dr12 = sim.simxGetObjectHandle(clientID, '/dr12',sim.simx_opmode_oneshot_wait)

        error_code, self.rollingJoint_fl = sim.simxGetObjectHandle(clientID, '/youBot/rollingJoint_fl',sim.simx_opmode_oneshot_wait)
        error_code, self.rollingJoint_rl = sim.simxGetObjectHandle(clientID, '/youBot/rollingJoint_rl',sim.simx_opmode_oneshot_wait)
        error_code, self.rollingJoint_rr = sim.simxGetObjectHandle(clientID, '/youBot/rollingJoint_rr',sim.simx_opmode_oneshot_wait)
        error_code, self.rollingJoint_fr = sim.simxGetObjectHandle(clientID, '/youBot/rollingJoint_fr',sim.simx_opmode_oneshot_wait)

        error_code, self.roll_joint_Arm0 = sim.simxGetObjectHandle(clientID,'/youBot/youBotArmJoint0',sim.simx_opmode_oneshot_wait)
        error_code, self.roll_joint_Arm1 = sim.simxGetObjectHandle(clientID,'/youBot/youBotArmJoint1',sim.simx_opmode_oneshot_wait)
        error_code, self.roll_joint_Arm2 = sim.simxGetObjectHandle(clientID,'/youBot/youBotArmJoint2',sim.simx_opmode_oneshot_wait)
        error_code, self.roll_joint_Arm3 = sim.simxGetObjectHandle(clientID,'/youBot/youBotArmJoint3',sim.simx_opmode_oneshot_wait)
        error_code, self.roll_joint_Arm4 = sim.simxGetObjectHandle(clientID,'/youBot/youBotArmJoint4',sim.simx_opmode_oneshot_wait)

        self.Kp = np.array([
            [2.0, 0, 0], 
            [0, -2.0, 0],
            [0, 0, -5]], dtype=float
        )
        self.Kd = np.array([
            [0, 0.1, 0], 
            [0.1, 0, 0],
            [0, 0, -0.3]], dtype=float
        )

        self.rrt = RRT()
    
    def getArmState(self, ):
        
        sols =  [
            sim.simxGetJointPosition(clientID,self.roll_joint_Arm0,sim.simx_opmode_oneshot_wait),
            sim.simxGetJointPosition(clientID,self.roll_joint_Arm1,sim.simx_opmode_oneshot_wait),
            sim.simxGetJointPosition(clientID,self.roll_joint_Arm2,sim.simx_opmode_oneshot_wait),
            sim.simxGetJointPosition(clientID,self.roll_joint_Arm3,sim.simx_opmode_oneshot_wait),
            sim.simxGetJointPosition(clientID,self.roll_joint_Arm4,sim.simx_opmode_oneshot_wait),]
        qs_list = []
        for res, qs in sols:
            if not res == sim.simx_return_ok:
                return None
            qs_list.append(qs)

        return qs_list
    
    def getPoseState(self, ):
        sols = [sim.simxGetObjectPosition(clientID, self.youb, -1, sim.simx_opmode_oneshot_wait),
        sim.simxGetObjectOrientation(clientID, self.youb, -1, sim.simx_opmode_oneshot_wait)]
        qs_list = []
        for res, qs in sols:
            if not res == sim.simx_return_ok:
                return None
            qs_list.append(qs)
        position, (alpha, beta, gamma) = qs_list
        R_original = euler_to_rotation_matrix(alpha, beta, gamma)
        T = np.array([[0, 0, -1],
                    [0, -1, 0],
                    [1, 0, 0]])
        R_modified = T.dot(R_original)
        alpha_new, beta_new, gamma_new = rotation_matrix_to_euler(R_modified)
        return [position, [alpha_new, beta_new, gamma_new]]
    
    def setPoseState(self, position, orientation):
        res = [sim.simxSetObjectPosition(clientID, self.youb, -1, position,  sim.simx_opmode_oneshot_wait),
               sim.simxSetObjectOrientation(clientID, self.youb, -1, orientation,  sim.simx_opmode_oneshot_wait)]
        for r in res:
            if not r == sim.simx_return_ok:
                return False
            
        return True
    
    def setWeelState(self, fl, rl, rr, fr):


        sim.simxSetJointTargetVelocity(clientID, self.rollingJoint_fl, fl, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(clientID, self.rollingJoint_rl, rl, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(clientID, self.rollingJoint_rr, rr, sim.simx_opmode_oneshot_wait)
        sim.simxSetJointTargetVelocity(clientID, self.rollingJoint_fr, fr, sim.simx_opmode_oneshot_wait)

    def setMovement(self, forwBackVel, leftRightVel, rotVel):
        fl = -forwBackVel-leftRightVel-rotVel
        rl = -forwBackVel+leftRightVel-rotVel
        rr = -forwBackVel-leftRightVel+rotVel
        fr = -forwBackVel+leftRightVel+rotVel

        self.setWeelState(fl, rl, rr, fr)
    
    def setArmState(self, qs):
        res = [
        sim.simxSetJointTargetPosition(clientID, self.roll_joint_Arm0, qs[0], sim.simx_opmode_oneshot_wait),
        sim.simxSetJointTargetPosition(clientID, self.roll_joint_Arm1, qs[1], sim.simx_opmode_oneshot_wait),
        sim.simxSetJointTargetPosition(clientID, self.roll_joint_Arm2, qs[2], sim.simx_opmode_oneshot_wait),
        sim.simxSetJointTargetPosition(clientID, self.roll_joint_Arm3, qs[3], sim.simx_opmode_oneshot_wait),
        sim.simxSetJointTargetPosition(clientID, self.roll_joint_Arm4, qs[4], sim.simx_opmode_oneshot_wait),]

        for r in res:
            if not r == sim.simx_return_ok:
                return False
            
        return True
    def getDR12State(self):
        sols = [sim.simxGetObjectPosition(clientID, self.dr12, -1, sim.simx_opmode_oneshot_wait),
        sim.simxGetObjectOrientation(clientID, self.dr12, -1, sim.simx_opmode_oneshot_wait)]
        qs_list = []
        for res, qs in sols:
            if not res == sim.simx_return_ok:
                return None
            qs_list.append(qs)
        return qs_list
    
    def lineFollow(self, line, position, orientaion, coa):
        (x_init,y_init), (x_goal,y_goal) = line
        x, y, z = position
        yaw = orientaion[2]

        vec1 = np.array([x_goal - x_init, y_goal - y_init])
        vec2 = np.array([x_goal - x, y_goal - y])
        vec1_hat = vec1 / np.linalg.norm(vec1)

        cross_track_error = np.cross(vec2, vec1_hat)

        course_angle = np.arctan2(vec1[1], vec1[0])
        course_angle_err = ssa(course_angle - yaw)

        dist = np.linalg.norm(vec2)
        vLR = cross_track_error * 10.0
        rotV = -20.0 * course_angle_err
        if dist < coa:
            self.moveToGoal((x_goal, y_goal, course_angle))

        self.setMovement(10.0, vLR, rotV)
        self.setArmState([0, 0, 0, 0, 0])
        return dist < coa

    def moveToGoal(self, goal):
        [x, y, z],orientation = self.getPoseState()
        gx, gy, gyaw = goal
        yaw = orientation[2]

        R = np.array([
            [np.cos(yaw), np.sin(yaw), 0],
            [-np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        e_global = np.array([
            gx - x, gy - y, ssa(gyaw - yaw)
        ])
        e_local = np.dot(R, e_global)
        V = (self.Kp @ e_local)
        self.setMovement(V[0], V[1], V[2])
    
    def exit(self,):
        sim.simxFinish(clientID)

    def moveArm(self, goal):
        armState = self.getArmState()
        postition, orientation = self.getPoseState()
        R = euler_to_rotation_matrix(*orientation)
        r_vec = np.array(goal) - np.array(postition)
        baselink_goal = R.T @ r_vec
        armbaselink_goal = r_vec - np.array([0.045, 0.0, 0.228])
        

        print('Rel Goal', armbaselink_goal)
        node = self.rrt.search(1500, armState, armbaselink_goal)
        path = []

        while node:
            path.append(node.state)
            node = node.parent
        path = path[::-1]

        for state in path:
            self.setArmState(state)

        return len(path) > 0