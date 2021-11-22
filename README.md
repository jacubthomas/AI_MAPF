# AI_MAPF
Multi-Agent Path Finding using A* search combined with a prioritized planning algorithm as well as a conflict-based search

Instruction set for this program are outlined "CSCI360_Project_3.pdf"

Running the python visualization will require an install of cmake. These installs may clear errors for you as well:  "pip install matplotlib" "brew install python-tk"

Task0 involves setting up the visualizer
Task1 involves defining the A* algorithm
Task2 involves defining the Priority Planning implementation
Task3 involves defining the Conflict-Based Search implementation

My solution is not 100% (they're is 1 elusive edge case in CBS and perhaps a constraint addition lacking in PP). This can be seen excel file "test/sum-of-costs.xlsx" which reflects my findings verse the goal outcome.

"terminal_cheat_sheet_testcases.rtf" splits the commands for running all 50 tests, using PP or CBS, at once. Simply copy paste to terminal and run from the respective task.
