import networkx as nx
import matplotlib.pyplot as plt
import math
import random
import numpy as np
import time



vector_size = 6
# This function will generate random solution 
def generate_random_solution():
    solution =[]
    for i in range(vector_size):
        solution.append(random.randint(0, 1))
    return solution

def mutate(solution):
    new_solution = solution.copy()
    change_index = random.randint(0, vector_size -1)
    if new_solution[change_index] != 1:
        new_solution[change_index] = 1
    else:
        new_solution[change_index] = 0
    return new_solution

def crossover(left, right):
    S1 = left [0:3] + right [3:6]
    S2 = right [0:3] + left [3:6]
    return (S1,S2)

            
def solution_validation(solution,num_nodes):   
    index =0
    for s in range(num_nodes):
            if index < num_nodes  and s != 0:
                path_index =  solution[index] + solution[index + 1]
                selected_path = path_dict[f"{s}-{0}"][path_index]
                # atleast one UAV must use relay because of the nature of consturcted network 
                if len(selected_path) > 2:
                    return True
                else:
                    False
                index += 1    
                             
    return True    



def min_total_RB(solution,num_nodes, RBMat):
    # Validate solution
   if not solution_validation(solution,num_nodes):
       return math.inf

   index =0
   total_rb =0
   
   for s in range(num_nodes):
        if index < num_nodes  and s != 0: # no need to cosnider ground station because we are not sending data from ground station
            path_index = solution[index] + solution[index + 1]
            selected_path = path_dict[f"{s}-{0}"][path_index]
            for k in range(len(selected_path)-1):
                i = selected_path[k]
                j = selected_path[k+1]
                total_rb = total_rb + RBMat[i][j]
                index +=1  
   return total_rb    
  
if __name__=='__main__':
    print("heuristic genetic algo for three UAV")
    start_time = time.time()
    num_nodes = 4 # incuding ground station
    
    # Links matrix bewteen two nodes if distance is given
    linksMat = [[0, 1, 1, 1],
                [1, 0, 0, 1],
                [1, 0, 0, 1],
                [1, 1, 1, 0]]

    # Required resource block between two nodes. It has been calculated based on distance

    RBMat = [[ 0,  5,  4, 16],
            [ 5,  0,  0,  9],
            [ 4,  0,  0, 11],
            [16,  9, 11,  0]]
    
    g = nx.Graph()
    for s in range(num_nodes):
        for d in range(num_nodes):
            # Source and destination pairs (s,d,) where s = UAV number and d = ground station which is zero
            if s != d and d == 0: 
                for i in range(num_nodes):
                    for j in range(num_nodes):
                        if linksMat[i][j] == 1:
                            g.add_edge(i, j)
   
   
    path_dict = dict()                  
    for s in range(num_nodes):
        for d in range(num_nodes):
            # Source and destination pairs (s,d,) where s = UAV number and d = ground station which is always zero
            if s != d and d == 0: 
                for i in range(num_nodes):
                    for j in range(num_nodes):
                        if linksMat[i][j] == 1:
                            # Considering only 3 paths from UAVs to Ground Station. It may very depends number of hops in the network 
                            paths = list(nx.shortest_simple_paths(g, s, d))[:3]
                            path_dict[f"{s}-{d}"] = paths
    
    print(path_dict)
    # Total population size
    population_size = 70
    population = []
    for i in range (population_size):
        population.append(generate_random_solution())
        
    population.sort(key=lambda x : min_total_RB(x,num_nodes, RBMat))

    number_of_generations = 0
    while number_of_generations <= 10000:  
        next_population = population[0:35]
        (S1, S2) = crossover(population[0], population[2])
        (S3, S4) = crossover(population[1], population[3])
        (S5, S6) = crossover(population[4], population[6])
        (S7, S8) = crossover(population[5], population[7])
        (S9, S10) = crossover(population[8], population[10])
        (S11, S12) = crossover(population[9], population[11])
        (S13, S14) = crossover(population[12], population[14])
        (S15, S16) = crossover(population[15], population[17])
        (S17, S18) = crossover(population[18], population[21])
        (S19, S20) = crossover(population[20], population[23])
        (S21, S22) = crossover(population[25], population[24])

        next_population += [S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12, S13, S14, S15, S16 , S17 , S18 , S19 , S20, S22]

        next_population.append(mutate(population[0]))
        next_population.append(mutate(population[3]))
        next_population.append(mutate(population[7]))
        next_population.append(mutate(population[12]))
        next_population.append(mutate(population[18]))
        next_population.append(mutate(population[25]))
        next_population.append(mutate(population[30]))
        next_population.append(mutate(population[32]))
        next_population.append(mutate(population[35]))
        next_population.append(mutate(population[38]))
        next_population.append(mutate(population[39]))
        next_population.append(mutate(population[45]))
        next_population.append(mutate(population[55]))
        
        next_population.sort(key=lambda x:min_total_RB(x,num_nodes, RBMat))
        population = next_population

        best_solution = min_total_RB(population[0],num_nodes, RBMat)
        # print(population[0])
        print('best_solution = ', best_solution)
        if best_solution == 23:
            print('We have found optimal solution!')
            break
        number_of_generations += 1
        
        
    print("Excution Time",(time.time() - start_time), "Seconds")

    # Creating Label for netwok
    labeldict = {}
    for s in range(num_nodes):
        if s == 0:
            labeldict[s] = "Ground Station"
        else:
            labeldict[s] = "UAV #" + str(s)  
            
    pos = nx.spring_layout(g)
    nx.draw(g,pos, labels = labeldict, with_labels = True)
    plt.show()

        