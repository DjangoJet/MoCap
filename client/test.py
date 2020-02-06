import numpy as np
import pickle

points = np.array([])
with open('points', 'rb') as f:
    points = pickle.load(f)

print(points[7][0])