#include <PrioritizedSearch.h>
#include <iostream>
using namespace std;

void PrioritizedSearch::preventGoalBlocking(list<Constraint> &constraints, int agent, int pathlength)
{
  // consider all other agents
  for(int i=0; i<a_star.ins.num_of_agents;i++)
  {
    // don't look at this->agent
      if(i!=agent)
    {
      // enact massive f penalty for crossing over a goal location after it's been reached
      for(int j=pathlength-2; j<100; j++)
      {
        Constraint c = Constraint(i, a_star.ins.goal_locations.at(agent), -1, j, VERTEX);
        constraints.push_back(c);
      }
    }
  }
}
vector<Path> PrioritizedSearch::find_solution() {

  int num_of_agents = a_star.ins.num_of_agents;
    vector<Path> paths(num_of_agents);

    // assign priority ordering to agents
    // By default, we use the index ordering of the agents where
    // the first always has the highest priority.
    list<int> priorities;
    for (int i = 0; i < num_of_agents; i++) {
        priorities.push_back(i);
    }

    list<Constraint> constraints;
    // plan paths
    for (int i : priorities) {
      
      // search for solution must return within interval (10 seconds by .h currently)
      if((clock() - start)/(double) CLOCKS_PER_SEC > end)
        {
            return paths; // return "No solution"
        }

      // agent 0 is top priority, no constraints 
      if(i == 0)
        paths[i] = a_star.find_path(i);
      else
        paths[i] = paths[i] = a_star.find_path(i, constraints);

        // for all agent_id w/ higher than i, add constraint for each edge/vertex from path i
        int time = 0;
        for(int step=0; step < paths[i].size(); step++)
        {  
          // Push constraint onto all agents w/ larger ids (less priority)
          Constraint c;
          for(int x=i+1; x< num_of_agents; x++)
          {
            // check if this is a goal state for x
            // if so, x can't arrive until after i
            if(a_star.ins.goal_locations.at(x) == paths[i].at(step))
            {
              // Vertex Constraint: blocks x until after i passes through
              for(int k=1; k<step;k++)
              {
                c = Constraint(x, paths[i].at(step), -1, k, VERTEX);
                constraints.push_back(c);
              }
              if(step > 0)
              {
                // backstep from spot to clear narrow corridor if present
                int backstep = step-1;
                list<int> path_blocker= a_star.ins.get_adjacent_locations(paths[i].at(backstep));

                // check for adj squares, if == 2 this is a passage way block - can't be here [0,t-1], repeat until adj > 2
                while(path_blocker.size() == 2)
                {
                  // disallow x to be on path until i has passed through
                  for(int k=1; k<backstep;k++)
                  {
                    c = Constraint(x, paths[i].at(backstep), -1, k, VERTEX);
                    constraints.push_back(c);
                  }
                  //prevents segfault
                  if(backstep-1 <= 0)
                    break;
                  path_blocker = a_star.ins.get_adjacent_locations(paths[i].at(backstep--));
                }
              }
            }
            // x cannot be on top of i
            c = Constraint(x, paths[i].at(step), -1, time, VERTEX);
            constraints.push_back(c);
            
            // x cannot move into i
            list<int> adj_loc = a_star.ins.get_adjacent_locations(paths[i].at(step));
            for(int loc: adj_loc)
            {
              c = Constraint(x, loc, paths[i].at(step), time, EDGE);
              constraints.push_back(c);
            }
          }
          time++;
        }

        // Check for goal block, and return no solution if present 
        preventGoalBlocking(constraints, i, paths[i].size());
        if(paths[i].size() > 100)
        {
          paths[i].resize(0);
        }
        
        if (paths[i].empty()) 
        {
          paths.resize(i);
          return paths;
        }
    }
    return paths;
}


