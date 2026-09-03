import numpy as np

with open('datafile.txt','r') as f: #reads the file into a list
    b = eval(f.read())

bArr = np.array(b) #turns the list to a np array
bMax = np.max(bArr) #finds the maximum
bMin = np.min(bArr) #finds the minumum
ind38 = np.where(bArr == 38)[0][0] #finds index for 38

values, counts = np.unique(bArr, return_counts=True) # gives counts of each value

mostIndex = np.argmax(counts) # finds the index of highest count
bMost = values[mostIndex]
howMany = counts[mostIndex]

bSorted = np.sort(bArr) # sorts the array

bEvenSorted = bSorted[bSorted % 2 == 0]  #only the even


#All print statements for Excercise #1
print("Assignment 1, Excercise 1\nJosiah Husmann SEED Lab\n")
print("1. Maximum:\t %d" % bMax)
print("2. Minimum:\t %d" % bMin)
print("3. Index of 38:\t %d" % ind38)
print("4. %d occured the most with %d counts." % (bMost,howMany))
print("5. Sorted list:", bSorted)
print("6. Even sorted list:", bEvenSorted)
