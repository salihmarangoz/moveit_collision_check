#!/usr/bin/env python

import torch
from torch import nn
import rospy
import numpy as np
from moveit_collision_check.srv import CheckCollision
from moveit_commander.robot import RobotCommander
import copy

class CollisionDistanceDataset(torch.utils.data.Dataset):
    def __init__(self):
        rospy.init_node('generate_collision_dataset')
        rospy.loginfo("Waiting for the check_collision service...")
        rospy.wait_for_service('/moveit_collision_check/check_collision')
        self.check_collision = rospy.ServiceProxy('/moveit_collision_check/check_collision', CheckCollision)
        self.robot_commander = RobotCommander(robot_description="robot_description_moveit_collision_check")
        self.robot_state = self.robot_commander.get_current_state()

    def __len__(self):
        return 256
    
    def get_num_joints(self):
        return len(self.robot_state.joint_state.name)

    def __getitem__(self, idx):
        robot_state = copy.deepcopy(self.robot_state)

        # randomize robot state
        new_position = []
        for joint_name in robot_state.joint_state.name:
            joint = self.robot_commander.get_joint(joint_name)
            new_position.append( np.random.uniform(joint.min_bound(), joint.max_bound()) )
            robot_state.joint_state.position = new_position
        res = self.check_collision(robot_state.joint_state)

        x = new_position
        if res.collision_state:
            y = 0.0
        else:
            #y = res.collision_distance
            y = 1.0
        #rospy.sleep(1.0)
        return np.array(x, dtype=np.float32), np.array([y], dtype=np.float32)

class NeuralNetwork(nn.Module):
    def __init__(self, num_inputs=20, layer_size=128):
        super(NeuralNetwork, self).__init__()
        self.layers = nn.Sequential(
            #nn.BatchNorm1d(num_inputs), 
            nn.Linear(num_inputs, layer_size), nn.BatchNorm1d(layer_size), nn.ReLU(),
            nn.Linear(layer_size, layer_size), nn.BatchNorm1d(layer_size), nn.ReLU(),
        )
        self.dist_layer = nn.Sequential(
            nn.Linear(layer_size, 1),
        )
        self.sign_layer = nn.Sequential(
            nn.Linear(layer_size, 1),
            #nn.Sigmoid()
        )
    def forward(self, x):
        x = self.layers(x)
        # dist = self.dist_layer(x)
        # sign = self.sign_layer(x)
        # return torch.cat((dist, sign), dim=1)
        return self.sign_layer(x)

def train(model, train_loader, opt, loss, epoch):
    model.train()
    for e in range(epoch):
        for batch_id, (data, label) in enumerate(train_loader):
            opt.zero_grad()
            pred = model(data)
            l = loss(pred, label)
            l.backward()
            opt.step()
            print("Loss:", float(l.detach()))

train_dataset = CollisionDistanceDataset()
train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=len(train_dataset))
model = NeuralNetwork(num_inputs=train_dataset.get_num_joints())
#loss = torch.nn.MSELoss()
loss =  torch.nn.BCEWithLogitsLoss()
opt = torch.optim.Adam(model.parameters())
#opt = torch.optim.SGD(model.parameters(), lr=0.01, momentum=0.5)

train(model, train_loader, opt, loss, 200) # <== set epoch here

print("model prediction / groundtruth")
true_count = 0
total_count = 0
with torch.no_grad():
    model.eval()
    for e in range(5): # TODO
        for x,y in train_loader:
            output = model(x)
            for a,b in zip(output, y):
                if float(a)>=0.0 and float(b)>=0.5:
                    true_count += 1
                elif float(a)<0.0 and float(b)<0.5:
                    true_count += 1
                total_count += 1
                print(np.array(a),np.array(b))

print("Total score:", true_count/total_count)

#sm = torch.jit.script(model)
#sm.save("mymodel.pt")