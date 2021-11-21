#include "CBS.h"
#include <iostream>
#include <queue>
#include <set>


Collision CBS::findCollision(CBSNode* node, Path a, int agent_index)
{
    node->paths.at(0);
    // look at every step in a
    // later, if path[a] is shorter than path[b], verify b againts last a location
    for(int step: a)
    {
        // examine every other path against a's path
        for(int i=0; i<node->paths.size(); i++)
        {
            Path p = node->paths.at(i);
            // prevents self-comparison
            if(a!=p)
            {
                //if path[a] is shorter than path[b]
                bool stops_short = false;
                if(a.size() < p.size())
                    stops_short = true;

                // compares each step
                for(int j=0; j<p.size();j++)
                {
                    // i/e prevents segfault, for |a| < |p|
                    if(stops_short)
                    {
                        // collision found
                        if(a.at(a.size()-1) == p.at(j))
                        {
                            return Collision(a, p, j, p.at(j), agent_index, i, 0);
                        }
                    }
                    else
                    {
                        // collision found
                        if(a.at(j) == p.at(j))
                        {
                            return Collision(a, p, j, p.at(j), agent_index, i, 0);
                        }
                        if(a.size() > j+ 1 && p.size() > j+1)
                        {
                            if(a.at(j) == p.at(j+1) && a.at(j+1) == p.at(j))
                            {
                                // cerr << "EDGE collision detected: " << a.at(j) << " : " << p.at(j) << " : " << j << endl;
                                return Collision(a, p, j+1, a.at(j), agent_index, i, p.at(j));
                            }
                        }
                    }
                }
            }
        }
    }
    return Collision();
}
void CBS::handleCollision(priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> * open,
                                                CBSNode* parentNode, Collision collision)
{
    // two iterations, one for each agent in collision
    for(int i=0; i<2; i++)
    {
        // create new node for each divergence to be (potentially) pushed onto pq "open"
        CBSNode* newNode = new CBSNode(*parentNode);
        all_nodes.push_back(newNode);
        
        // function repeats for each agent, so these keep track/differentiate of who's who
        int agent_index;
        Path agent_path;
        if(i==0)
        {
            agent_index = collision.idx_a;
            agent_path = collision.path_a; 
        }
        else
        {
            agent_index = collision.idx_b;
            agent_path = collision.path_b;
        }
        
        // collision can occur with an agent who stopped at their goal prior to time of collision
        int time;
        if(agent_path.size()-1 < collision.time)
            time = agent_path.size()-1;

        // Vertex-based collision
        if(collision.type == 0)
        {
            // constrain the agent from being at scene at and, if stopped prior, leading up to event 
            for(int j=time; j<=collision.time; j++)
            {
                Constraint c = Constraint(agent_index, collision.location, -1, j, VERTEX);
                newNode->constraints.push_back(c);
            }
        } 
        // Edge-based collision - they ran into eachother
        else
        {
            // int random_assign = rand() % 4;
            int random_assign = rand() % 4;
            if(random_assign == 0)
            {

                // cerr << "agent_index : " << agent_index << " : " << newNode->paths.at(agent_index).at(0) << " : " 
                // << collision.type << " : " << collision.location << " : " << collision.time << endl;
                Constraint c = Constraint(agent_index, collision.type, collision.location, collision.time, EDGE);
                newNode->constraints.push_back(c);
                // Constraint c = Constraint(agent_index, collision.location, -1, collision.time,  VERTEX);
                // newNode->constraints.push_back(c);
            }
            else if(random_assign == 1)
            {
                Constraint c = Constraint(agent_index, collision.location, collision.type, collision.time,  EDGE);
                newNode->constraints.push_back(c);
            }
            else if(random_assign == 2)
            {
                Constraint c = Constraint(agent_index, collision.type, collision.location, collision.time-1,  EDGE);
                newNode->constraints.push_back(c);
            }
            else
            {
                Constraint c = Constraint(agent_index, collision.location, collision.type, collision.time-1,  EDGE);
                newNode->constraints.push_back(c);
            }
        }
        // find new path for agent given these constraints
        Path newPath = a_star.find_path(agent_index, newNode->constraints);

        // can only push onto "open" if path reaches destination
        if(newPath.size() != 0)
        {
            // updates old path with new
            Path* updatePath = &newNode->paths.at(agent_index);
            updatePath->resize(newPath.size());

            for(int k=0; k<newNode->paths.at(agent_index).size();k++)
            {
                updatePath->at(k) = newPath.at(k);
                // cerr << newNode->paths.at(agent_index).at(k) << " , ";
            }

            // Update cost of Q
            newNode->cost = getSumOfCosts(newNode);
            open->push(newNode);
        }
    }
}
int CBS::getSumOfCosts(CBSNode* node)
{
    int sum = 0;
    for(Path p : node->paths)
    {
        sum += p.size()-1;
    }
    return sum;
}
vector<Path> CBS::find_solution() {
    priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> open; // open list
    /* generate the root CBS node */
    auto root = new CBSNode();
    all_nodes.push_back(root);   // whenever generating a new node, we need to
                                 // put it into all_nodes
                                 // so that we can release the memory properly later in ~CBS()
    // find paths for the root node
    root->paths.resize(a_star.ins.num_of_agents);
    for (int i = 0; i < a_star.ins.num_of_agents; i++) {

        // TODO: if you change the input format of function find_path()
        //  you also need to change the following line to something like
        // root->paths[i] = a_star.find_path(i, list<Constraint>());

        // Line 2: find initial paths w/o constraints 
        root->paths[i] = a_star.find_path(i);
        if (root->paths[i].empty()) {
            cout << "Fail to find a path for agent " << i << endl;
            return vector<Path>(); // return "No solution"
        }
    }
    // Line 3: compute the cost of the root node
    for (const auto& path : root->paths)
        root->cost += (int)path.size() - 1;

    // Line 4: put the root node into open list
    open.push(root);
    CBSNode* top;
    int stuck_count = 0;
    int stuck;
    // Line 5
    while (!open.empty()) {
        // TODO: implement the high-level of CBS
        // Line  6: examine the node w/ smallest cost
        top = open.top();
        open.pop();
        // cerr << top->cost << endl;

        set<int> paths_observed;

        bool collision_free = true;

        // consider all paths
        for(int i=0; i<top->paths.size();i++)
        {
            int path_index = -1;
            int lowest_cost = INT32_MAX;
            Path lowest_path;
            // locates the lowest, unconsidered path
            for(int j=0; j<top->paths.size();j++)
            {
                // if lower than lowest and not already observed
                if(top->paths.at(j).size() < lowest_cost &&  paths_observed.count(j) == 0)
                {
                    path_index = j;
                    lowest_cost = top->paths.at(j).size();
                    lowest_path = top->paths.at(j);
                }
            }
            // marks path as considered
            paths_observed.insert(path_index);

            // Line 7
            Collision collision = findCollision(top, lowest_path, path_index);
            if(collision.time != 0)
            {
                collision_free = false;
                int time = collision.time;
                // cerr << "Collision observed: "  << collision.location << " @ " << time  << 
                // " involving: " << collision.idx_a << " & " << collision.idx_b << endl;
                handleCollision(&open, top, collision);
                // cerr << "open.size() AFTER: " << open.size() << endl;
                open.pop();
                break;
            }
        }
        // Line (8,9) no collisions observed through out paths
        if(collision_free)
        {
            cout <<  "collision free: " <<  collision_free << endl;
            return top->paths;
        }
    }

    return vector<Path>(); // return "No solution"
}

CBS::~CBS() {
    // release the memory
    for (auto n : all_nodes)
        delete n;
}
