# Quick start
```bash
./run.sh
```
This will build and run the unit tests

# Questions 1 and 2
Can be found in drivetrain.h and drivetrain.cpp  
solution strategy is a BST

# Unit Tests
The [ ] denote the stating positions of the front and rear gears.  
Only the ratio is auto checked.  
Any fail will cause main() to return 1 immediately.

# Key assumption
Input lists are both sorted descending and have no repeats

# Concerns / Future Growth
- The heap is rarely used in embedded systems; they have little dynamic memory and it is up against important areas. No OS safety
- - better solution would be use a BST as an array on the stack
- Recursion is dangerous. Embedded systems can have smaller memory spaces and are more prone to stack overflow
- - loop implementation would be better.
- Better use of const in the functions
- #ifdef to turn of color prints; current out put is ignoring for automated test pipe lines

# Online references used
BST c  
https://www.codesdope.com/blog/article/binary-search-tree-in-c/

sorted array into BST  
https://www.geeksforgeeks.org/sorted-array-to-balanced-bst/

# Hardware question
I know very little on selecting the switch for a switching power supply. Here is my thought path:

 
Page 8 of the control unit's data sheet outlines the selection params for the external components:
"The main selection criteria for
the power MOSFET are the threshold voltage V GS(TH) and
the “on” resistance R DS(ON) , reverse transfer capacitance
and total gate charge."

## Threshold voltage
- so the minimum voltage (negativ) to turn on the switch. Also to turn off the switch, need to get past this number.
- The IRF7202 matches the 70170 at -1.0V
- The NDS9405 is at -0.5V, which to me means the switch might not turn off.

## R DS (ON)
- The resistance across the switch, which I think we want as low as possible for best efficacy.
- The IRF7202 is about 0.25 ohms.
- The NDS9405 is about 0.065 ohms.
- So the IRF7202 would have more power loss, driving down the total efficiency of the switching converter.

## My pick
Unless after asking a team member about the lower threshold voltage; I would select the IRF7202 even thought it has a lower efficiency.  
I'm less worried that it would outright break the circuit.
