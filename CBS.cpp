#include "CBS.h"
#include <iostream>
#include <queue>
#include <set>


Collision CBS::findCollision(CBSNode* node, Path a, int agent_index)
{
    set<int> sofar;
    // examine every other path against a's path
    for(int i=0; i<node->paths.size(); i++)
    {
        // check for collisions against paths in random order
        int randomizeme = rand()%node->paths.size();
        while(sofar.count(randomizeme)!=0)
        {
            randomizeme = rand()%node->paths.size();
        }
        sofar.insert(randomizeme);

        // get path[i]
        Path p = node->paths.at(randomizeme);

        // prevents self-comparison
        if(a!=p)
        {
            //if path[a] is shorter than path[b]
            bool stops_short_a = false;
            if(a.size() < p.size())
                stops_short_a = true;

            // look at every step in a
            for(int k=0; k<a.size();k++)
            {
                // prevents segfault
                if(p.size() > k)
                {
                    // vertex collision
                    if(a.at(k) == p.at(k))
                    {
                        return Collision(a, p, k, p.at(k), agent_index, randomizeme, 0);
                    }
                    // prevents segfault
                    if(a.size() > k+1 && p.size() > k+1)
                    {
                        // edge collision
                        if(a.at(k+1) == p.at(k) && a.at(k) == p.at(k+1))
                        {
                            return Collision(a, p, k+1, a.at(k), agent_index, randomizeme, p.at(k));
                        }
                    }
                }
                else
                {
                    // vertex collision
                    if(a.at(k) == p.at(p.size()-1))
                    {
                        return Collision(a, p, k, p.at(p.size()-1), agent_index, randomizeme, 0);
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
        // Edge-based collision - they ran into each other
        else
        {
            if(i==0)
            {
                Constraint c = Constraint(agent_index, collision.location, collision.type, collision.time,  EDGE);
                newNode->constraints.push_back(c);
            }
            else if(i==1)
            {
                Constraint c = Constraint(agent_index, collision.type, collision.location, collision.time, EDGE);
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
            }

            // Update cost of Q
            newNode->cost = getSumOfCosts(newNode);
            open->push(newNode);
        }
        // else
        // {
        //     cerr << "cannot find path for agent  " << agent_index << endl;
        // }
    }
}

// sums the cost of all paths for optimized solution comparison
int CBS::getSumOfCosts(CBSNode* node)
{
    int sum = 0;
    for(Path p : node->paths)
    {
        sum += p.size()-1;
    }
    return sum;
}

vector<Path> CBS::find_solution() 
{
    priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> open; // open list
    /* generate the root CBS node */
    auto root = new CBSNode();
    all_nodes.push_back(root);   // whenever generating a new node, we need to
                                 // put it into all_nodes
                                 // so that we can release the memory properly later in ~CBS()
    // find paths for the root node
    root->paths.resize(a_star.ins.num_of_agents);
    for (int i = 0; i < a_star.ins.num_of_agents; i++) {

        // find initial paths w/o constraints 
        root->paths[i] = a_star.find_path(i);
        if (root->paths[i].empty()) {
            cout << "Fail to find a path for agent " << i << endl;
            return vector<Path>(); // return "No solution"
        }
    }
    // compute the cost of the root node
    for (const auto& path : root->paths)
        root->cost += (int)path.size() - 1;

    // put the root node into open list
    open.push(root);
    CBSNode* top;

    // iterate through priority queue until optimal solution is found, none remain, or 10 seconds elapse
    while (!open.empty()) {

        // 10-second search threshold
        if((clock() - start)/(double) CLOCKS_PER_SEC > end)
        {
            return vector<Path>(); // return "No solution"
        }

        // examine the node w/ smallest cost
        top = open.top();
        open.pop();

        // if thiis holds after loop, optimal solution found
        bool collision_free = true;

            // look for a collision in all paths of node
            for(int j=0; j<top->paths.size();j++)
            {
                Collision collision = findCollision(top, top->paths.at(j), j);
                if(collision.time != 0)
                {
                    handleCollision(&open, top, collision);
                    collision_free = false;
                }
            }
        // no collisions observed through out paths
        if(collision_free)
        {
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
