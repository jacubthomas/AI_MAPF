#include "AStarPlanner.h"
#include <queue>
#include <unordered_map>
#include <algorithm> // reverse
#include <iostream>
using namespace std;

ostream& operator<<(ostream& os, const Path& path)
{
    for (auto loc : path) 
    {
        os << loc << " ";
    }
    return os;
}

Path AStarPlanner::make_path(const AStarNode* goal_node) const 
{
    Path path;
    const AStarNode* curr = goal_node;
    while (curr != nullptr) 
    {
        path.push_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
    return path;
}


// helper function, reduces redundant code in findpath()
bool AStarPlanner::evaluateConstraints(Constraint abide1, Constraint abide2, const list<Constraint>& constraints)
{
    bool safe = true;
    for(Constraint con : constraints)
    {
        // cerr << "CONSTRAINT: " <<get<0>(con) << " : " << get<1>(con) << " : " << get<2>(con) << " : " << get<3>(con) << " : " << get<4>(con) << endl;
        // cerr << "ABIDE2: "<< get<0>(abide2) << " : " << get<1>(abide2) << " : " << get<2>(abide2) << " : " << get<3>(abide2) << " : " << get<4>(abide2) << endl;
        if(get<0>(con) == get<0>(abide1) &&
            get<1>(con) == get<1>(abide1) &&
            get<2>(con) == get<2>(abide1) &&  
            get<3>(con) == get<3>(abide1) &&
            get<4>(con) == get<4>(abide1))
        {
            safe = false;
        }
        if(get<0>(con) == get<0>(abide2) &&
            get<1>(con) == get<1>(abide2) &&
            get<2>(con) == get<2>(abide2) &&  
            get<3>(con) == get<3>(abide2) &&
            get<4>(con) == get<4>(abide2))
        {
            safe = false;
        }
    }
    return safe;
}

bool AStarPlanner::isGoalBlocked(int agent_i, int current_location)
{
    list<int> adj_to_curr = ins.get_adjacent_locations(current_location);
    // consider locations reachable from current square
    for(int tile : adj_to_curr)
    {
        // consider all agents w/ priority to agent_i
        for(int j=0; j<agent_i; j++)
        {
            // agent_i is next to agent_j's (w/ priority) goal square
            if(tile == ins.goal_locations.at(j))
            {
                // check if agent_j's goal is next to agent_i's goal
                list<int> adj_to_j_goal = ins.get_adjacent_locations(tile);
                list<int> adj_to_i_goal = ins.get_adjacent_locations(ins.goal_locations.at(agent_i));
                for(int tile_j: adj_to_j_goal)
                {
                    if(tile_j == ins.goal_locations[agent_i])
                    {
                        // check if there is only one path into agent_i's goal
                        if(adj_to_i_goal.size() <= 1)
                            return true;
                    }
                }
            }
        }
    }
    return false;
}

Path AStarPlanner::find_path(int agent_id, const list<Constraint>& constraints) {
    int start_location = ins.start_locations[agent_id];
    int goal_location = ins.goal_locations[agent_id];

    // Open list
    priority_queue<AStarNode*, vector<AStarNode*>, CompareAStarNode> open;

    // Unordered map is an associative container that contains key-value pairs with unique keys.
    // The following unordered map is used for duplicate detection, where the key is the location of the node.
    // unordered_map<int, AStarNode*> all_nodes;
    // TODO: For Task 1, you need to replace the above line with
    unordered_map<pair<int, int>, AStarNode*, hash_pair> all_nodes;

    int h = ins.get_Manhattan_distance(start_location, goal_location); // h value for the root node
    auto root = new AStarNode(start_location, 0, h, nullptr, 0);
    open.push(root);
    Path path;

    // Consider nodes as they surface in the priority queue
    while (!open.empty()) {
        auto curr = open.top();
        open.pop();

        // goal test
        if (curr->location == goal_location) {
            path = make_path(curr);
            break;
        }
        // no possible path
        if(isGoalBlocked(agent_id, curr->location))
            break;
            
        int best_f_location = 1000;
        list<AStarNode*> astars;
        // int no_solution_tracker = 0;
        // generate child nodes
        for (auto next_location : ins.get_adjacent_locations(curr->location)) 
        {

            // link locations with timesteps
            pair<int,int> temp = make_pair(next_location, curr->timestep+1);
            auto it = all_nodes.find(temp);

            // for(AStarNode* node: all_nodes)
            // the location has not been visited before
            if (it == all_nodes.end()) 
            {
                // do a check here that path abides all constraints (one for vertex,  other for edge)
                Constraint abide1 = Constraint(agent_id, next_location, -1, curr->timestep+1, VERTEX);
                Constraint abide2 = Constraint(agent_id, curr->location, next_location, curr->timestep+1, EDGE);

                // safe will be updated if this node is constrained
                bool safe = evaluateConstraints(abide1, abide2, constraints);
                if(safe)
                {
                    // update values for child node creation
                    int next_g = curr->g + 1;
                    int next_h = ins.get_Manhattan_distance(next_location, goal_location);
                    int next_ts = curr->timestep + 1;
                    auto next = new AStarNode(next_location, next_g, next_h, curr, next_ts);

                    // idea: capture best direction to move, and verify not blocked to prevent deadlock sitting
                    if(next_g +  next_h <  best_f_location)
                        best_f_location = next_location;

                    // astars will capture all "safe" moves and push in follow-on loop
                    astars.push_back(next);
                    // push new node onto pq 
                    open.push(next);

                    // add node to um for duplicate detection
                    all_nodes[temp] = next;
                }
            }
            // Note that if the location has been visited before,
            // next_g + next_h must be greater than or equal to the f value of the existing node,
            // because we are searching on a 4-neighbor grid with uniform-cost edges.
            // So we don't need to update the existing node.
        }
            // additional child node for stalling in place
            pair<int,int> temp = make_pair(curr->location, curr->timestep+1);
            auto it = all_nodes.find(temp);

            // Check for vertex and edge constraint
            Constraint abide3 = Constraint(agent_id, curr->location, -1, curr->timestep+1, VERTEX);
            Constraint abide4 = Constraint(agent_id, curr->location, curr->location, curr->timestep+1, EDGE);

            // safe will be updated if this node is constrained
            bool safe = evaluateConstraints(abide3, abide4, constraints);

            // the location has not been visited before
            if (it == all_nodes.end() && safe) {
            
                // updated elements for child node creation
                int next_g = curr->g+1;
                int next_h = curr->h;
                int next_ts = curr->timestep + 1;
                auto next = new AStarNode(curr->location, next_g, next_h, curr, next_ts);

                // push new node onto pq 
                open.push(next);
                // add node to um for duplicate detection
                all_nodes[temp] = next;
            }
    }
    // release memory
    for (auto n : all_nodes)
        delete n.second;

    return path;
}
