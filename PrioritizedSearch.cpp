#include <PrioritizedSearch.h>
#include <iostream>

using namespace std;

/*
  this can be improved upon, given time permits:
  extend goal blocking to check/prevent lesser agents from blocking path, further away from higher 
  agent's goal than the implemented adjacent square; that is, provided, it still blocks the path.
  imagine a scenario w/ a narrow corridor

  s_b s_a ... ... ... g_a ... ... ... g_b
  .@. .@. ... .@. .@. .@. .@. .@. .@. .@. 

*/
void PrioritizedSearch::preventGoalBlocking(list<Constraint> &constraints, int agent, int pathlength)
{
  // // consider all paths into agent's goal
  // list<int> adj_locs = a_star.ins.get_adjacent_locations(a_star.ins.goal_locations.at(agent));

  // // agent has one (or no) path into goal
  // if(adj_locs.size() < 2)
  // {
  //   // consider all agents w/ lesser priority
  //   for(int i=agent+1; i<a_star.ins.num_of_agents;i++)
  //   {
  //     // compare path into goal against agent's goal placement
  //     int adj_tile = adj_locs.front();
  //     if(i!=agent && adj_tile == a_star.ins.goal_locations.at(i))
  //     {
  //       // block agent w/ lesser priority from reaching goal prior to this->agent w/ higher priority
  //       Constraint c = Constraint(i, adj_tile, -1, pathlength-3, VERTEX);
  //       constraints.push_back(c);
  //     }
  //   }
  // }
  // else // no goal blocking here// no goal blocking here
  // { 
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
  // }
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
      // TODO: Transform already planned paths into constraints
      //  Replace the following line with something like paths[i] = a_star.find_path(i, constraints);
      
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
              // blocks x until after i passes through
              for(int k=1; k<step;k++)
              {
                c = Constraint(x, paths[i].at(step), -1, k, VERTEX);
                constraints.push_back(c);
              }
              
              // check for adj squares, if == 2, this is a passage way block - can't be here [0,t-1], repeat until adj > 2
              int backstep = step-1;
              list<int> path_blocker= a_star.ins.get_adjacent_locations(paths[i].at(backstep));
              while(path_blocker.size() == 2)
              {
                for(int k=1; k<backstep;k++)
                {
                  c = Constraint(x, paths[i].at(backstep), -1, k, VERTEX);
                  constraints.push_back(c);
                }
                path_blocker = a_star.ins.get_adjacent_locations(paths[i].at(backstep--));
              }
              for(int tiles: path_blocker)
              {
                if(tiles == paths[i].at(backstep-1))
                {
                  cerr << "found it: " <<  endl;
                  cerr << tiles << " : " << backstep -1  << endl;
                  c = Constraint(x, tiles, -1, backstep, VERTEX);
                  constraints.push_back(c);
                }
              }
            }
            // x cannot be on top of i
            c = Constraint(x, paths[i].at(step), -1, time, VERTEX);
            constraints.push_back(c);
            // cerr << "constraint: " << x << " : " << paths[i].at(step) << " : "<< -1 << " : " << time<< " : VERTEX" << endl; 
            
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
        preventGoalBlocking(constraints, i, paths[i].size());
        if (paths[i].empty()) {
          paths.resize(i);
          return paths;
        }
    }
    return paths;
}


