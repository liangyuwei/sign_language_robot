from __future__ import division
from numpy.linalg import norm
import math
import numpy as np
import copy

class GD_Optimizer:

  def __init__(self, costFunc, bounds, maxiter, options=None):

    self.costFunc = costFunc
    self.bounds = bounds
    self.cost_history = []
    self.step_history = []
    self.maxiter = maxiter

    if 'alpha' in options: # learning rate
      self.alpha = options['alpha']
    else:
      self.alpha = 0.001
    
    if 'epsilon' in options: # for calculation of gradient
      self.epsilon = options['epsilon']
    else:
      self.epsilon = 0.0001

    if 'precision' in options: # for stopping criterion
      self.precision = options['precision']
    else:
      self.precision = 0.01


  def evaluate(self, x):

    # for no heuristics   
    cost = self.costFunc(x)
    if cost == -1: cost = 10 # being -1 means that an infeasible point is encountered
    return cost
    

    # for heuristics
    '''
    tmp = self.costFunc(x)
    if tmp == -1: return -1 # infeasible point
    else:
      l_min_time, r_min_time, l_goal, r_goal = tmp
      cost = max(l_min_time, r_min_time)
      if l_min_time > r_min_time: # right arm have spare time
        grad_heuristic = np.array(l_goal) - np.array(r_goal) # 
      else: # left arm have spare time
        grad_heuristic = np.array(r_goal) - np.array(l_goal)

      grad_heuristic = grad_heuristic / norm(grad_heuristic)        
      return cost, grad_heuristic.tolist()
    '''

  
  def update(self, x_last, gradient):#, grad_heuristic):

    # rescale the magnitude of heuristic gradient
    '''
    scale = 0.01#2.0#0.01 #0.2#1.0 # too much heuristic grad could jeopardize the original gradient!!!
    grad_heuristic = np.array(grad_heuristic)
    gradient = np.array(gradient)
    grad_heuristic = scale * norm(gradient) * grad_heuristic
    '''

    # update
    x_next = []
    for j in range(len(x_last)):
      x_next.append( x_last[j] - self.alpha * gradient[j]) #(gradient[j]+grad_heuristic[j]) )
      # deal with bounds
      if x_next[j] < self.bounds[j][0]:
        x_next[j] = self.bounds[j][0]
      if x_next[j] > self.bounds[j][1]:
        x_next[j] = self.bounds[j][1]      

    return x_next



  def train(self, x0):

    #window_size = 0
    last_cost = -1 # denoting the first iteration
    precision = self.precision 
    cur_x = copy.deepcopy(x0)
    i = 0
    while i<self.maxiter:
      print('========== Iteration ' + str(i) + ' of GD ===========')
      # compute gradient
      gradient = []      
      for j in range(len(cur_x)):
        # display debug message
        print('Computing gradient for dim ' + str(j+1) + '/' + str(len(cur_x)) + ' ...')
        # prepare variables
        x_plus = copy.deepcopy(cur_x)
        x_plus[j] += self.epsilon
        x_minus = copy.deepcopy(cur_x)
        x_minus[j] -= self.epsilon
        # compute J
        #J_plus, _ = self.evaluate(x_plus)
        #J_minus, _ = self.evaluate(x_minus)
        J_plus = self.evaluate(x_plus)
        J_minus = self.evaluate(x_minus)
        # gradient 
        gradient.append( (J_plus-J_minus)/(2*self.epsilon) )
      print('computed gradient is: ' + str(gradient))
      # update the next point
      #cur_cost, grad_heuristic = self.evaluate(cur_x)
      #print('heuristic gradient is: ' + str(grad_heuristic))
      cur_cost = self.evaluate(cur_x)
      next_x = self.update(cur_x, gradient)#, grad_heuristic)

      # record cost history
      # how to avoid infeasible positions????
      #cost, _ = self.evaluate(cur_x)
      print('Recording cost history...')
      self.cost_history.append(cur_cost)
      print('current cost: ' + str(cur_cost) + '\n')

      # stopping criterion
      #step = np.sqrt( np.sum( (np.array(cur_x) - np.array(next_x)) ** 2) )
      #cur_cost = self.evaluate(cur_x)
      step = abs(cur_cost - last_cost)
      self.step_history.append(step)
      print('Update step of this iteration: ' + str(step))
      if step <= precision and last_cost != -1:
        #window_size += 1
        #print('Precision on update step size is achieved ' + str(window_size) + '/3')
        #if window_size >= 3: # precision is met for three iterations
        #  print('Precision is met for 3 iterations, terminate the optimization process...')
        #  cost = self.evaluate(cur_x)
        #  return cost, cur_x        
        print('Precision on cost change is achieved, terminate the process now...')
        return cur_cost, cur_x
      else:    
        #window_size = 0 # reset count 
        cur_x = next_x
        last_cost = cur_cost
    
      # next iteration
      i += 1

    print('Unfortunately precision is not met, reaching end of GD.')
    
    return cost, cur_x # not the optimal solution!!!



