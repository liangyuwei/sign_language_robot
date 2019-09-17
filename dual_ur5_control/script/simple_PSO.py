
#--- IMPORT DEPENDENCIES ------------------------------------------------------+

from __future__ import division
from numpy.linalg import norm
import random
import math
import copy
import numpy as np

#--- COST FUNCTION ------------------------------------------------------------+

# function we are attempting to optimize (minimize)
def func1(x):
    total=0
    for i in range(len(x)):
        total+=x[i]**2
    return total

#--- MAIN ---------------------------------------------------------------------+
# data structure(class) for a single particle to store corresponding information
class Particle:
    def __init__(self,x0, options=None):
        self.position_i=[]          # particle position
        self.velocity_i=[]          # particle velocity
        self.pos_best_i=[]          # best position, individual, so far
        self.err_best_i=-1          # best error, individual, so far
        self.err_i=-1               # error individual
        self.options = options      # w, c1, c2


        # initialization of position and velocity
        for i in range(0,num_dimensions):
            self.velocity_i.append(random.uniform(-1,1))
            self.position_i.append(x0[i])


    # evaluate current fitness
    def evaluate(self,costFunc):

        tmp=costFunc(self.position_i)

        # check if it's a feasible solution
        if tmp != -1: #(still compute for the first time even it's not feasible!!! cost much time to initialize with feasible points)
            self.err_i = tmp
            # check to see if the current position is an individual best
            if self.err_i<self.err_best_i or self.err_best_i==-1:
                self.pos_best_i=copy.deepcopy(self.position_i)
                self.err_best_i=self.err_i
 
       
    # update new particle velocity
    def update_velocity(self,pos_best_g):
        if self.options is None:
            w=0.5       # constant(could be dynamically changing!! need modifications in that case) inertia weight (how much to weigh the previous velocity)
            c1=1        # cognitive acceleration constant
            c2=2        # social acceleration constant
        else:
            w=self.options['w']
            c1=self.options['c1']
            c2=self.options['c2']

        for i in range(0,num_dimensions):
            r1=random.random()
            r2=random.random()
            
            vel_cognitive=c1*r1*(self.pos_best_i[i]-self.position_i[i])
            vel_social=c2*r2*(pos_best_g[i]-self.position_i[i])
            self.velocity_i[i]=w*self.velocity_i[i]+vel_cognitive+vel_social


    # update the particle position based on new velocity updates, with bounds
    def update_position(self,bounds):
        for i in range(0,num_dimensions):
            self.position_i[i]=self.position_i[i]+self.velocity_i[i]
            
            # adjust maximum position if necessary
            if self.position_i[i]>bounds[i][1]:
                self.position_i[i]=bounds[i][1]

            # adjust minimum position if neseccary
            if self.position_i[i]<bounds[i][0]:
                self.position_i[i]=bounds[i][0]
        

class PSO():
    def __init__(self, costFunc, x0, bounds, num_particles, maxiter, verbose=False, options=None):
        global num_dimensions

        num_dimensions=len(x0[0])
        err_best_g=-1                   # best error for group
        pos_best_g=[]                   # best position for group

        # record cost history
        self.cost_history = []    
        self.leader_step_history = []

        # criterion threshold
        eps1 = 0.05
        eps2 = 0.6 # how much is good???
        #eps3 = 0.06
        #delta = 0.6 # 60% of particles
        cnt_violate = 0 # number of times(consecutive) that slope of cost violates the threshold


        # establish the swarm, initialize particles 
        print('Initializing particle swarm...')
        #import pdb
        #pdb.set_trace()
        if len(x0) != num_particles:
            print("[ERROR]: The number of initial particles: " + str(len(x0)) + " is not in consistent with the given num_particles: " + str(num_particles))
            return False
        swarm=[]
        for i in range(0,num_particles):
            swarm.append(Particle(x0[i], options))


        # compute diameter of the set of initial points for later use(maximum swarm radius)
        diam_x0 = -1
        for i in range(0, num_particles-1):
            for j in range(i+1, num_particles):
                dist_ij = np.sqrt( np.sum( ( np.array(x0[i])-np.array(x0[j]) )**2 ) )
                if dist_ij > diam_x0: diam_x0 = dist_ij
  

        # begin optimization loop
        i=0
        last_err = -1
        while i<maxiter:
            if verbose: print('iter: ' + str(i) + ', best solution: ' + str(err_best_g) + '\n')
            # cycle through particles in swarm and evaluate fitness
            for j in range(0,num_particles):
                # evaluate fitness
                swarm[j].evaluate(costFunc)
                # determine if current particle is the best (globally), and update immediately
                if swarm[j].err_i<err_best_g or err_best_g==-1:
                    pos_best_g=list(swarm[j].position_i)
                    err_best_g=float(swarm[j].err_i)


            # record cost history
            self.cost_history.append(err_best_g)
            if i == 0: # first iteration
                self.leader_step_history.append(0)
            else:
                dist = norm(np.array(last_pos_best_g) - np.array(pos_best_g))
                self.leader_step_history.append(dist)
            # record the current best position
            last_pos_best_g = pos_best_g


            # cycle through swarm and update velocities and position
            for j in range(0,num_particles):
                swarm[j].update_velocity(pos_best_g)
                swarm[j].update_position(bounds)
           

            ## stopping criterion
            if i != 0: # not the first iteration
                slope = (err_best_g - last_err) / err_best_g
                print('[DEBUG]: slope of cost function is: ' + str(slope))

                # the first stopping criterion: if not improving much
                if abs(slope) < eps1:
                    print('Best err not improving...')
                    cnt_violate += 1

                    if cnt_violate >= 4:
                        # the second stopping criterion: if all particles close to the global best particle
                        Rmax = -1
                        for m in range(num_particles):
                            dist_p_gb = np.sqrt( np.sum( (np.array(swarm[m].position_i) - np.array(pos_best_g)) **2 ) )
                            if dist_p_gb > Rmax: Rmax = dist_p_gb
                        print('[DEBUG]: ratio of swarm radius: ' + str(Rmax / diam_x0))
                        if Rmax / diam_x0 < eps2: 
                            print('>>> Best err not improving much for ' + str(cnt_violate) +' iterations, check the radius now...')                        
                            break
                   
                else:
                    cnt_violate = 0

            # update the last err information
            last_err = err_best_g

            # next iteration
            i+=1

        # print final results
        print('\nFINAL SOLUTION:')
        print('   > ' + str(pos_best_g))
        print('   > ' + str(err_best_g) + '\n')
        
        # store the results
        self.err_best_g = err_best_g
        self.pos_best_g = pos_best_g

    def result(self):

        return self.err_best_g, self.pos_best_g




if __name__ == "__main__":
    main()

#--- RUN ----------------------------------------------------------------------+
def main():
  initial=[5,5]               # initial starting location [x1,x2...]
  bounds=[(-10,10),(-10,10)]  # input bounds [(x1_min,x1_max),(x2_min,x2_max)...]
  options = {'c1':1, 'c2':2, 'w':0.5}
  PSO(func1, initial, bounds, num_particles=15, maxiter=30, verbose=True, options=options)

#--- END ----------------------------------------------------------------------+
