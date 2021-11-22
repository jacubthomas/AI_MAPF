
#pragma once
#include "AStarPlanner.h"


class PrioritizedSearch {
public:
  vector<Path> find_solution();
  PrioritizedSearch(const MAPFInstance& ins): a_star(ins) 
  {
    start = clock();
  }
  void preventGoalBlocking(list<Constraint> &constraints, int agent, int pathlength);
  clock_t start;
  double end = 10;
private:
  AStarPlanner a_star;
};

/*
* 1) Design a MAPF instance for which prioritized planning does not find an (op- timal or suboptimal)
* collision-free solution for a given ordering of the agents. Explain why your MAPF instance meets the
* requirement.
* 
* Instance 1 is reflected in exp2_3.paths. Prioritized planning does not find a collision-free solution
* due to the inconvenient ordering of the  agents. Here, agent 1 is planned first, it's priority is
* higher than that of agent 2. Combining this with the fact that agent 1's goal location blocks the
* only path into agent 2's goal, no solution exists given this approach.
              4 7
              @ @ @ @ @ @ @
              @ . . . . . @
              @ @ @ . @ @ @
              @ @ @ @ @ @ @
              2
              1 2 1 4
              1 1 1 5
*
* Instance 2) is adapted from exp2_3.paths. Prioritized planning finds a suboptimal collision-free
* solution due to the inconvenient ordering of the  agents. Here, agent 1 is planned first, it's 
* priority ishigher than that of agent 2. Combining this with the fact that agent 1's goal location
* blocks the more cost-effective into agent 2's goal, P.P returns a solution which costs exceed the
* minimum; agent 2 will have to make two extra moves to go around agent 1, which would not occur if
* the ordering was switched.
              4 7
              @ @ @ @ @ @ @
              @ . . . . . @
              @ @ @ . . . @
              @ @ @ @ @ @ @
              2
              1 2 1 4
              1 1 1 5
*/