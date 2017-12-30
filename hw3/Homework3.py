from __future__ import division
# CSCI 3302: Homework 3 -- Clustering and Classification
# Implementations of K-Means clustering and K-Nearest Neighbor classification

# Nic Williams

import pickle
import numpy as np
import math
from collections import Counter



class KMeansClassifier(object):

  def __init__(self):
    self._cluster_centers = [] # List of points representing cluster centers
    self._data = [] # List of datapoints (lists of real values)

  def add_datapoint(self, datapoint):
    self._data.append(datapoint)

  def fit(self, k):
    # Fit k clusters to the data, by starting with k randomly selected cluster centers.
    # HINT: To choose reasonable initial cluster centers, you can set them to be in the same spot as random (different) points from the dataset
    # TODO Follow convergence procedure to find final locations for each center
    # TODO Add each of the 'k' final cluster_centers to the model (self._cluster_centers)
    self._cluster_centers = []
    local_cluster = []
    converge = 0 #loop termination condition initialization
    for i in range(0, k): #initialize the centroids randomly
      c = np.random.choice(len(self._data))
      self._cluster_centers.append(self._data[c])
      i += 1
    while converge != k: #update until center points converge
      converge = 0
      for datapoint in self._data: # numbers are easier to work with for grouping
         i = 0
         local_cluster.append(self.classify(datapoint))
         i += 1
      for i in range(0, k): #center needs to be re-calculated
          x_sum = y_sum = count = 0
          for j in range(0, len(self._data)): #new center is the mean of data points
            if local_cluster[j] == i:
              x_sum += self._data[j][0]
              y_sum += self._data[j][1]
              count += 1
              j += 1
          centroid_x = x_sum/count
          centroid_y = y_sum/count
          if (centroid_x == self._cluster_centers[i][0]) & (centroid_y == self._cluster_centers[i][1]): #for all k-points converging breaks loop
            converge += 1
          else: #update if no convergence occurs
            self._cluster_centers[i][0] = centroid_x
            self._cluster_centers[i][1] = centroid_y
          i += 1
    return self._cluster_centers

  def classify(self,p):
    # Given a data point p, figure out which cluster it belongs to and return that cluster's ID (its index in self._cluster_centers)
    # TODO Find nearest cluster center, then return its index in self._cluster_centers
    closest_cluster_index = 0
    minimum_distance = np.sqrt((p[0]-self._cluster_centers[0][0])**2 + (p[1]-self._cluster_centers[0][1])**2) #euclidan distance calculation
    for i in range(0, len(self._cluster_centers)): #iterate through distance calulations between centroids and p, set to shortest distance
      temp = np.sqrt((p[0]-self._cluster_centers[i][0])**2 + (p[1]-self._cluster_centers[i][1])**2)
      if minimum_distance > temp:
         minimum_distance = temp
         closest_cluster_index = i
      i += 1
    return closest_cluster_index

class KNNClassifier(object):

  def __init__(self):
    self._data = [] # list of (data, label) tuples

  def clear_data(self):
    self._data = []

  def add_labeled_datapoint(self, data_point, label):
    self._data.append((data_point, label))

  def classify_datapoint(self, data_point, k): # followed sentdex youtube tutorial on KNN https://www.youtube.com/watch?v=GWHG3cS2PKc
      #TODO: Perform k_nearest_neighbor classification, set best_label to majority label for k-nearest points
      label_counts = {} # Dictionary mapping "label" => count
      best_label = None
      distances = []
      for groups in self._data:
        euclidean_distance = np.linalg.norm(np.array(groups[0])-np.array(data_point))
        distances.append([euclidean_distance, groups[1]])
      votes = [i[1] for i in sorted(distances)[:k]]
      best_label = Counter(votes).most_common(1)[0][0]
      return best_label

def print_and_save_cluster_centers(classifier, filename):
  for idx, center in enumerate(classifier._cluster_centers):
    print ("Cluster %d, center at: %s".format((idx, str(center))))


  f = open(filename,'wb')
  pickle.dump(classifier._cluster_centers, f)
  f.close()

def read_data_file(filename): #had to convert DOS linefeeds to UNIX so tinkered with this and the pkl file with a script
  try:                        #link: https://stackoverflow.com/questions/2613800/how-to-convert-dos-windows-newline-crlf-to-unix-newline-n-in-a-bash-script/19702943#19702943
    f = open(filename, 'rb')
    data_dict = pickle.load(f)
    f.close()
  except IOError:
    print('An error occured trying to read the file.')
  return data_dict['data'], data_dict['labels']

def main():
  # read data file
  data, labels = read_data_file('hw3_data.pkl')

  # data is an 'N' x 'M' matrix, where N=number of examples and M=number of dimensions per example
  # data[0] retrieves the 0th example, a list with 'M' elements
  # labels is an 'N'-element list, where labels[0] is the label for the datapoint at data[0]


  ########## PART 1 ############
  # perform K-means clustering
  kMeans_classifier = KMeansClassifier()
  for datapoint in data:
    kMeans_classifier.add_datapoint(datapoint) # add data to the model

  kMeans_classifier.fit(4) # Fit 4 clusters to the data


  # plot results
  print ('\n'*2)
  print ("K-means Classifier Test")
  print ('-'*40)
  print ("Cluster center locations:")
  print_and_save_cluster_centers(kMeans_classifier, str("hw3_kmeans_Nic.pkl"))


  print ('\n'*2)


  ########## PART 2 ############
  print ("K-Nearest Neighbor Classifier Test")
  print ('-'*40)

  # Create and test K-nearest neighbor classifier
  kNN_classifier = KNNClassifier()
  k = 2

  correct_classifications = 0
  # Perform leave-one-out cross validation (LOOCV) to evaluate KNN performance
  for holdout_idx in range(len(data)):
    # Reset classifier
    kNN_classifier.clear_data()

    for idx in range(len(data)):
      if idx == holdout_idx: continue # Skip held-out data point being classified

      # Add (data point, label) tuples to KNNClassifier
      kNN_classifier.add_labeled_datapoint(data[idx], labels[idx])

    guess = kNN_classifier.classify_datapoint(data[holdout_idx], k) # Perform kNN classification
    if guess == labels[holdout_idx]:
      correct_classifications += 1.0

  print ("kNN classifier for k=%d" % k)
  print ("Accuracy: %g" % (correct_classifications / len(data)))
  print ('\n'*2)




if __name__ == '__main__':
  main()
