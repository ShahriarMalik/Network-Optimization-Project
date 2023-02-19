import numpy as np
from gurobipy import *
import time

start_time = time.time()

# Data
# num of nodes incuding ground station
num_nodes = 6
    
 # Links matrix bewteen two nodes if distance is given
linksMat = [[0, 1, 1, 1, 1, 1],
            [1, 0, 0, 1, 1, 0],
            [1, 0, 0, 1, 1, 1],
            [1, 1, 1, 0, 1, 0],
            [1, 1, 1, 1, 0, 0],
            [1, 0, 1, 0, 0, 0]]

# Required resource block between two nodes, It has been calculated based on distance

RBMat = [[ 0,  8,  4, 16, 42,  5],
        [ 8,  0,  0,  9,  4,  0],
        [ 4,  0,  0, 11, 11,  4],
        [16,  9, 11,  0,  3,  0],
        [42,  4, 11,  3,  0,  0],
        [ 5,  0,  4,  0,  0,  0]]


m = Model("optimization for Five UAV using Gurobi")


#Decision variables 

#Traffic routing decision variable
x = tupledict() # x[s,d,i,j] = 1 if traffic from s to d traverses link (i,j)(binary)


# Source and destination pairs (s,d,) where s = UAV number and d = ground station which is zero
for s in range(num_nodes):
    for d in range(num_nodes):
        if s != d and d == 0: 
            for i in range(num_nodes):
                for j in range(num_nodes):
                    if linksMat[i][j] == 1: # for all links (i,j) that exist
                        x[s,d,i,j] = m.addVar(vtype = GRB.BINARY, name = 'x^' + str(s) + ',' + str(d) + "_(" + str(i) + "," + str(j) + ")")

#Objective function: minimize total number of RB
total_rb = 0
for s in range(num_nodes):
    for d in range(num_nodes):
        if s != d and d == 0: # for all pairs (s,d) where s != d
            for i in range(num_nodes):
                for j in range(num_nodes):
                    if linksMat[i][j] == 1:
                        total_rb += x[s,d,i,j] * RBMat[i][j]
m.setObjective(total_rb, GRB.MINIMIZE)


# multi-commodity flow constraint
# Source and destination pairs (s,d,) where s = UAV number and d = ground station which is zero
for s in range(num_nodes):
    for d in range(num_nodes):
        if s != d and d == 0:
            for i in range(num_nodes):
                incoming_links = 0
                outgoing_links = 0
                for j in range(num_nodes):
                    if linksMat[i][j]:
                        incoming_links += x[s,d,i,j] 
                        outgoing_links += x[s,d,j,i]
                    desired_value = 0
                    if i == s:
                        desired_value = 1
                    if i == d:
                        desired_value = -1
                m.addConstr(outgoing_links - incoming_links == desired_value)
m.optimize()

print("Minimum RB required for the network: "+ str(m.objVal))
for v in m.getVars():
        if v.x:
            print(f"{v.varName}: {v.x}")
    
    
print("Excution Time",(time.time() - start_time), "Seconds")
