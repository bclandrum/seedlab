# Excercise 1 Demo 1 Seed Lab - Bradley Landrum
import numpy as np #import numpy for computation
# read in datafile.txt as described in assignment
with open('datafile.txt','r') as f: 
    b = eval(f.read())
b_array = np.array(b) # converts list to numpy array
b_maximum = np.max(b_array) # finds maximum value
b_minimum = np.min(b_array) # find the minimum value
index_38 = np.where(b_array == 38)[0][0] # index value for 38

values, counts = np.unique(b_array, return_counts=True) # finds counts of each value in b_array
max_index = np.argmax(counts) # finds index w/ highest count
most_num = values[max_index] # most common value
num_times = counts[max_index] # number of times repeated

b_sort = np.sort(b_array) # creates sorted array
b_even = b_sort[b_sort %2 == 0] # creates even sorted array

# print
print("Assignment #1 Excercise #1\nBradley Landrum\nTeam 1")
print("\n1. Maximum = %d"%b_maximum)
print("\n2. Minimum = %d"%b_minimum)
print("\n3. Index of 38 = %d"%index_38)
print("\n4. %d occured most frequently, exactly %d times"%(most_num,num_times))
print(f"\n5. Sorted array: {b_sort}")
print(f"\n6. Even sorted array: {b_even}")




