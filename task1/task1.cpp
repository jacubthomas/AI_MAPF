#include <iostream>
#include <fstream>
#include "MAPFInstance.h"
#include "AStarPlanner.h"
#include <tuple>

// idea: if goal locations are adjacent, someone needs to steer clear
// checks if j is adj to i. if so, returns tile value of j; otherwise, -1
int iADJj(MAPFInstance ins, int i, int j, int SorG)
{   
    int i_pos;
    int j_pos;
    list<int> adj_pos;
    // checking start_positions
    if(SorG == 1)
    {
        i_pos = ins.start_locations.at(i);
        j_pos = ins.start_locations.at(j);
        adj_pos = ins.get_adjacent_locations(i_pos);
    } 
    // checking goal_positions
    else 
    {
        i_pos = ins.goal_locations.at(i);
        j_pos = ins.goal_locations.at(j);
        adj_pos = ins.get_adjacent_locations(i_pos);
    }
    for(int x  = 0; x <adj_pos.size(); x++)
    {
        int temp = adj_pos.front();
        if(j_pos == temp)
        {
            return temp;
        }
        adj_pos.pop_front();
    }
    return -1;
}

void manhattanStall(MAPFInstance ins, list<Constraint> &constraints, int i, int goal, bool blocked)
{
    int md = ins.get_Manhattan_distance(ins.start_locations.at(i), goal);
    list<int> adj_loc = ins.get_adjacent_locations(ins.start_locations.at(i));
        for(int loc: adj_loc)
        {
            for(int j=0; j<md/2; j++)
            {
                Constraint c = Constraint(i, loc, -1, j, VERTEX);
                constraints.push_back(c);
            }
        }
}
void protectGoalZones(MAPFInstance ins, list<Constraint> &constraints)
{
    //  idea: unless a's goal blocks throughput, give priority to shortest md. other's must wait.
    // couple this with, agents cannot pass over goals unless it blocks path.
    int deadzones[ins.num_of_agents];
    bool goalblocks[ins.num_of_agents];
    for(int i=0; i<ins.num_of_agents; i++)
    {
        int goal = ins.goal_locations.at(i);
        // if greater than 2, does not block
        if(ins.get_adjacent_locations(goal).size() > 2) goalblocks[i] = false;
        else goalblocks[i] = true;
        deadzones[i] = goal;
        if(!goalblocks[i])
        {
            for(int j=0; j<ins.num_of_agents; j++)
            {
                if(i!=j)
                {
                    for(int k=0; k<100; k++)
                    {
                        Constraint c = Constraint(j, deadzones[i], -1, k, VERTEX);
                        constraints.push_back(c);
                        // cerr << "add : " << j << " : " << deadzones[i] << " : " << -1 << " : " << k  << " : "<<  VERTEX << endl;
                    }
                }
            }
        }
        // stall based on md
        manhattanStall(ins, constraints, i, goal, goalblocks[i]);
    }
}
void corneredAdj(MAPFInstance ins, list<Constraint> &constraints)
{
    // idea: cornered agents should leave after neighbors clear, if also trapped by surrounding agents
    // how: adj_loc size <= 2 && adj_loc contains startpos of another agent.
    for(int i=0; i<ins.num_of_agents; i++)
    {
        int start_pos = ins.start_locations.at(i);
        list<int> neighbors = ins.get_adjacent_locations(start_pos);

        if(neighbors.size() <= 2)
        {
            // cerr << "neigbors <= 2 for: " << i << endl;
            for(int j=0; j<ins.num_of_agents; j++)
            {
                // prevents self-check
                if(i!=j)
                {
                    for(int n : neighbors)
                    {
                        // checks for agents, j,  who start next to i.
                        if(n == ins.start_locations.at(j))
                        {
                            // cerr << "agent j: " << j << " is in adj start pos: " << n << endl;
                            int n_size = neighbors.size();
                            // stall i for, arbitrary, 5 seconds
                            for(int k=0; k< n_size; k++)
                            {
                                int n_front = neighbors.front();
                                cerr << "i: " << i << n_front <<endl;
                                Constraint c = Constraint(i, start_pos, n_front, 1, EDGE);
                                constraints.push_back(c);
                                c = Constraint(i, start_pos, n_front, 2, EDGE);
                                constraints.push_back(c);
                                c = Constraint(i, start_pos, n_front, 3, EDGE);
                                constraints.push_back(c);
                                c = Constraint(i, start_pos, n_front, 4, EDGE);
                                constraints.push_back(c);
                                c = Constraint(i, start_pos, n_front, 5, EDGE);
                                constraints.push_back(c);
                                neighbors.pop_front();
                            }
                        }
                    }
                }
            }
        }
    }
}
// check for alt path, excluding starting position tiles
bool checkForAltPath(MAPFInstance ins, list<Constraint> &constraints, int agent)
{
    list<int> adjacent = ins.get_adjacent_locations(agent);
    int numtiles = adjacent.size();
    if(numtiles == 1)
        return false;

    // consider all non-blocked adjacent squares
    for(int n : adjacent)
    {   
        // if freetile survives all agents, square is not a start location
        bool freetile = true;
        for(int j=0; j<ins.num_of_agents; j++)
        {
            if(agent!=j)
            {
                if(n == ins.start_locations.at(j))
                    freetile = false;
            }
        }
        if(freetile)
            return true;
    }
    return false;
}
int main(int argc, char *argv[]) {
    MAPFInstance ins;
    string input_file = argv[1];
    string output_file = argv[2];
    if (ins.load_instance(input_file)) {
        ins.print_instance();
    } else {
        cout << "Fail to load the instance " << input_file << endl;
        exit(-1);
    }
    AStarPlanner a_star(ins);
    vector<Path> paths(ins.num_of_agents);
    list<Constraint> constraints;
    if(ins.num_of_agents > 2)
        corneredAdj(ins, constraints);
    protectGoalZones(ins, constraints);
    // consider every agent i
    for (int i = 0; i < ins.num_of_agents; i++) {
        // cerr << "constraints outer loop" << endl;

        Constraint c;
        // TODO: specify constraints here
        // ⟨ai,x,−1,t,VERTEX⟩ ⟨ai,x,y,t,EDGE)


        // ensure for each agent i, with regard to all other agents j
        for(int j=0; j< ins.num_of_agents; j++)
        {   
            // cerr << "constraints inner loop" << endl;
            //  prevent comparing agent to self
            if( j != i)
            {
               // premise:if i and j start adjacent to each other
               int temp = iADJj(ins, i, j, 1);
               if(temp != -1)
               {
                    // idea: i cannot move into j immediately
                    // how (1): if alt path available, must take that
                    if(checkForAltPath(ins, constraints, i))
                    {
                        for(int x=0; x<10; x++)
                        {
                            c = Constraint(i, ins.start_locations.at(i), temp, x, EDGE);
                            // cerr << "add : " << i << " : " << ins.start_locations.at(i) << " : " << temp << " : " << 1  << " : "<<  EDGE << endl;
                            constraints.push_back(c);
                        }
                    }
                    // how (2): only one path available; simply stall for a second
                    else
                    {
                        c = Constraint(i, ins.start_locations.at(i), temp, 1, EDGE);
                        // cerr << "add : " << i << " : " << ins.start_locations.at(i) << " : " << temp << " : " << 1  << " : "<<  EDGE << endl;
                        constraints.push_back(c);
                    }

                    
                    if(ins.num_of_agents <= 2)
                    {
                    c = Constraint(j, ins.start_locations.at(j), -1, 1, VERTEX);
                    // cerr << "add : " << j << " : " << ins.start_locations.at(j) << " : " << -1 << " : " << 1  << " : "<<  VERTEX << endl;
                    constraints.push_back(c);
                    }

                    // idea: j cannot move towards i immediately
                    // how (1): if alt path available, must take that
                    if(checkForAltPath(ins, constraints, i))
                    {
                        for(int x=0; x<10; x++)
                        {
                            c = Constraint(j, ins.start_locations.at(j), ins.start_locations.at(i), x, EDGE);
                            // cerr << "add : " << i << " : " << ins.start_locations.at(i) << " : " << temp << " : " << 1  << " : "<<  EDGE << endl;
                            constraints.push_back(c);
                        }
                    }
                    // how (2): only one path available; simply stall for a second
                    else
                    {
                        c = Constraint(j, ins.start_locations.at(j), ins.start_locations.at(i), 1, EDGE);
                        // cerr << "add : " << j << " : " << ins.start_locations.at(j) << " : " << ins.start_locations.at(i) << " : " << 1  << " : "<<  EDGE << endl;
                        constraints.push_back(c);
                    }

                    // lesser md cannot arrive at goal before greater
                    int md_i = ins.get_Manhattan_distance(ins.start_locations.at(i), ins.goal_locations.at(i));
                    int md_j = ins.get_Manhattan_distance(ins.start_locations.at(j), ins.goal_locations.at(j));
                    int gmd = 0;
                    int gma = -1;
                    if(md_i > md_j){ gmd = md_i; gma = j; } 
                    else if(md_i < md_j) { gmd = md_j; gma = i; }
                    // if they're equal in md, this never runs
                    for(int x=1; x < gmd; x++)
                    {
                        c = Constraint(gma, ins.goal_locations.at(gma), -1, x, VERTEX);
                        // cerr << "add : " << gma << " : " << ins.goal_locations.at(gma) << " : " << "-1 : " << x  << " : "<<  VERTEX << endl;
                        constraints.push_back(c);
                    }
               }
                // premise:if i and j goals are adjacent to each other
                temp = iADJj(ins, i, j, 2);
                if(temp != -1)
               {
                    // lesser manhattan distance cannot arrive at goal before greater
                    int md_i = ins.get_Manhattan_distance(ins.start_locations.at(i), ins.goal_locations.at(i));
                    int md_j = ins.get_Manhattan_distance(ins.start_locations.at(j), ins.goal_locations.at(j));

                    // greater manhattan distance value
                    int gmd = 0;

                    // greater manhattan distance variable
                    int gma = -1;
                    if(md_i > md_j){ gmd = md_i; gma = j;} 
                    else if(md_i < md_j) { gmd = md_j; gma = i;}

                    // if they're equal in md, this never runs
                    // if not, prevent lesser manhattan distance from reaching goal prior to greater
                    for(int x=1; x < gmd; x++)
                    {
                        c = Constraint(gma, ins.goal_locations.at(gma), -1, x, VERTEX);
                        // cerr << "add : " << gma << " : " << ins.goal_locations.at(gma) << " : " << "-1 : " << x  << " : "<<  VERTEX << endl;
                        constraints.push_back(c);
                    }
                    // Make lesser move out of way
                    for(int x=gmd-2; x < gmd; x++)
                    {
                        c = Constraint(gma, ins.goal_locations.at(gma)-1, -1, x, VERTEX);
                        // cerr << "add : " << gma << " : " << ins.goal_locations.at(gma)-1 << " : " << "-1 : " << x  << " : "<<  VERTEX << endl;
                        constraints.push_back(c);
                    }
               }
            }
        }

        paths[i] = a_star.find_path(i, constraints);

        if (paths[i].empty()) {
            cerr << "Fail to find any solutions for agent " << i << endl;
            return 0;
        }
    }

    // print paths
    cout << "Paths:" << endl;
    int sum = 0;
    for (int i = 0; i < ins.num_of_agents; i++) {
        cout << "a" << i << ": " << paths[i] << endl;
        sum += (int)paths[i].size() - 1;
    }
    cout << "Sum of cost: " << sum << endl;

    // save paths
    ofstream myfile (output_file.c_str(), ios_base::out);
    if (myfile.is_open()) {
        for (int i = 0; i < ins.num_of_agents; i++) {
            myfile << paths[i] << endl;
        }
        myfile.close();
    } else {
        cout << "Fail to save the paths to " << output_file << endl;
        exit(-1);
    }

    return 0;
}
