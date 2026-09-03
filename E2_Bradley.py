# Excercise 2 Demo 1 Seed Lab - Bradley Landrum
# String checker finite state machine : checks for abcd in a user inputted string of letters
def state_0(character): #define state 0
    return state_a if character=='a' else state_0 # checks if the character is a
def state_a(character): #checks if a, or b, or if it should reset to 0
    if character == 'b':
        return state_b
    elif character == 'a':
        return state_a
    else:
        return state_0
def state_b(character): # checks if a, c, or if it should reset to 0
    if character == 'c':
        return state_c
    elif character == 'a':
        return state_a
    else:
        return state_0
def state_c(character): # checks if a, or if abcd, or if it should be reset to 0
    if character == 'a':
        return state_a
    elif character == 'd':
        return state_d
    else:
        return state_0
def state_d(character): # confirm abcd in string, or if a, or if it should be reset to 0
    print("abcd is in the string!\n")
    if character == 'a':
        return state_a
    else:
        return state_0

abcd_string = input("Input a string of letters: ") # prompts user for string
state=state_0 #sets default state as state_0

for character in abcd_string: # main loop to check states and go through the string
    state=state(character)