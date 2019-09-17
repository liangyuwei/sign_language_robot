from __future__ import division
import math
import numpy as np
import copy

class GD_Optimizer:

  def __init__(self, costFunc, bounds, maxiter, options=None):

    self.err = -1
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

    return self.costFunc(x)

  
  def update(self, x_last, gradient):

    x_next = []
    for j in range(len(x_last)):
      x_next.append( x_last[j] - self.alpha * gradient[j] )
      # deal with bounds
      if x_next[j] < self.bounds[j][0]:
        x_next[j] = self.bounds[j][0]
      if x_next[j] > self.bounds[j][1]:
        x_next[j] = self.bounds[j][1]      

    return x_next



  def train(self, x0):

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
        J_plus = self.evaluate(x_plus)
        J_minus = self.evaluate(x_minus)
        # gradient 
        gradient.append( (J_plus-J_minus)/(2*self.epsilon) )
      print('computed gradient is: ' + str(gradient))
      # update the next point
      next_x = self.update(cur_x, gradient)

      # record cost history
      # how to avoid infeasible positions????
      cost = self.evaluate(cur_x)
      print('Recording cost history...')
      self.cost_history.append(cost)
      print('current cost: ' + str(cost) + '\n')

      # stopping criterion
      step = np.sqrt( np.sum( (np.array(cur_x) - np.array(next_x)) ** 2) )
      self.step_history.append(step)
      print('Update step of this iteration: ' + str(step))
      if step <= precision:
        print('Precision on update step size is achieved, terminate GD now...')
        cost = self.evaluate(cur_x)
        return cost, cur_x
      else:    
        cur_x = next_x
    
      # next iteration
      i += 1

    print('Unfortunately precision is not met, reaching end of GD.')
    
    return cost, cur_x # not the optimal solution!!!



