import numpy as np
import random
from fk import *
from utils import *
from tqdm import tqdm
import matplotlib.pyplot as plt
class Node:
    def __init__(self, state) -> None:
        self.state = state
        self.parent = None



class RRT:
    def __init__(self, ):
        self.nodes = None
        self.poseNodes = None
        self.spaceRange = {'low' : [-0.1, -0.5, -0.5], 'high' : [0.5, 0.5, 0.5]}
        self.angleRange = {
            'low': [-np.radians(169), -np.radians(65), -np.radians(150), -np.radians(102.5), -np.radians(167.5)],
            'high': [np.radians(169), np.radians(90), np.radians(146), np.radians(102.5), np.radians(167.5)]
        }

        self.actionRange = {
            'low': [-(h - l) / 50 for l, h in zip(self.angleRange['low'], self.angleRange['high'])],
            'high': [(h - l) / 50 for l, h in zip(self.angleRange['low'], self.angleRange['high'])]
        }


    def nearestNodeIdx(self, pose):
        return np.argmin(np.linalg.norm(self.poseNodes[:len(self.nodes)] - pose,axis = 1))
    
    def sampleSpace(self, goal):
        if np.random.random() < 0.1:
            return goal
        return np.random.uniform(**self.spaceRange)
    
    def sampleState(self, state):
        inBound = False
        while not inBound:
            rndAction = np.random.uniform(**self.actionRange)
            inBound, newState = self.inBound(state=state, action=rndAction)

        return newState
    
    def inBound(self, state, action):
        new_state = [ssa(s + ds) for s,ds in zip(state, action)]

        for min_r, max_r in zip(self.angleRange['low'], self.angleRange['high']):
            for angle in new_state:
                if not min_r < angle < max_r:
                    return False, new_state
        return True, new_state
    
    def checkCollision(self, state):
        return False
    
    def checkGoal(self, pose, goal):
        pose = np.array(pose)
        return np.linalg.norm(goal - pose) < 0.05
    
    def search(self, iter, start_state, goal):
        start_node = Node(start_state)
        self.poseNodes = np.zeros((iter + 1, 3))
        i = 0
        for i in tqdm(range(iter)):
            angles = np.random.uniform(**self.angleRange).tolist()
            newPose = transform2(*angles)[:3, -1]
            self.poseNodes[i] = newPose
            if self.checkGoal(newPose, goal):
                print(newPose, goal, np.linalg.norm(goal - newPose))
                newNode = Node(angles)
                newNode.parent = start_node
                return newNode
        return None
            
            
    
    def searchRRT(self, iter, start_state, goal):
        self.nodes = [Node(start_state)]
        self.poseNodes = np.zeros((iter + 1, 3))
        goal = np.array(goal)

        for _ in tqdm(range(iter)):
            rnd_pose = self.sampleSpace(goal = goal)
            nearest_node_idx = self.nearestNodeIdx(rnd_pose)
            nearest_node: Node = self.nodes[nearest_node_idx]
            
            newState = self.sampleState(state = nearest_node.state)
            if not self.checkCollision(newState):
                newStatePose = transform2(*newState)[:3, -1]
                newNode = Node(state=newState)
                newNode.parent = nearest_node

                idx = len(self.nodes)
                self.poseNodes[idx] = newStatePose
                self.nodes.append(newNode)

                if self.checkGoal(newStatePose, goal):
                    print(newStatePose, goal, np.linalg.norm(goal - newStatePose))
                    return newNode
                
        return None
    
if __name__ == '__main__':
    obs = RRT()
    goal = [0.3, 0.0, 0.0]
    node = obs.searchRRT(1500, [0, 0, 0, 0, 0], goal)
    nodePose = np.zeros(3)
    if node:
        nodePose = transform2(*node.state)[:3, -1]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract x, y, and z coordinates from the points array
    x = obs.poseNodes[:, 0]
    y = obs.poseNodes[:, 1]
    z = obs.poseNodes[:, 2]

    # Plot the points
    ax.scatter(x, y, z, c='r', marker='o')
    ax.scatter(goal[0], goal[1], goal[2], c = 'g', marker = '*')
    ax.scatter(nodePose[0], nodePose[1], nodePose[2], c = 'k', marker = '*')
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Scatter Plot')

    # Show plot
    plt.show()


    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # ax.set_title('3D Scatter Plot')

    # path = []
    # temp = node
    # while temp:
    #     path.append(temp.state)
    #     temp = temp.parent

    # path = path[::-1]

    # for state in path:
    #     pose = transform2(*state)[:3, -1]
    #     ax.scatter(pose[0], pose[1], pose[2], c='b', marker='o')
    #     plt.pause(0.1)

    # plt.show()
