import numpy as np
import random
from math import sqrt, pi, exp

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories
    def __init__(self):
        self.classes = ['left', 'keep', 'right']
    
    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, X):
        return X
    
    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").
    def train(self, X, Y):
        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''
        # TODO: implement code.
        self.X_arr, self.Y_arr = np.array(X), np.array(Y)
        unique, cnt = np.unique(Y, return_counts=True)
        self.priors = cnt / len(Y)
        
        self.means = np.array([self.X_arr[np.where(self.Y_arr==i)].mean(axis=0) for i in self.classes])
        self.stds = np.array([self.X_arr[np.where(self.Y_arr==i)].std(axis=0) for i in self.classes])
        
        return self
            

    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
        # TODO: implement code.
        probas = np.zeros(len(self.classes))
        # Calculate Gaussian probability for each variable based on the
        # mean and standard deviation calculated in the training process.
        for i in range(len(self.classes)):
            proba = 1
            for k in range(self.X_arr.shape[1]):
                # Multiply all the probabilities for variables, and then
                # normalize them to get conditional probabilities.
                proba *= gaussian_prob(observation[k], self.means[i][k], self.stds[i][k])
#             proba *= self.priors[i]
            probas[i] = proba
        res = probas / probas.sum()
        
        # Return the label for the highest conditional probability.
        return self.classes[res.argmax(axis=0)]
    


