# Assignment 1, Excercise 2
# Josiah Husmann



def state0(cha):
    return aState if cha=='a' else state0 #checks if the cha is a

def aState(cha): #checks if the cha is a, b, or needs to go back to state0
    if cha == 'b': 
        return bState
    elif cha == 'a': 
        return aState
    else: 
        return state0

def bState(cha): #checks if the cha is a, c, or needs to go back to state0
    if cha == 'c': 
        return cState
    elif cha == 'a': 
        return aState
    else: 
        return state0 

def cState(cha): #checks if the cha is a, needs to go back to state 0, or abcd is in the string
    if cha == 'd': 
        print('abcd is contained in the string')
        return dState
    elif cha == 'a': 
        return aState
    else: 
        return state0

def dState(cha): #checks if the cha is a, b, or needs to go back to state0
    return aState if cha=='a' else state0

abcdStr = input("Input a string: ") #asks for the string
state = state0 # initializes to state 0
for cha in abcdStr: #runs through each char in abcdStr
    state = state(cha)

