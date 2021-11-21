#pragma once
#include "AStarPlanner.h"
#include <tuple>
#include <queue>

struct CBSNode {
  // Line 1: initially empty constraints list
  list<Constraint> constraints;
  vector<Path> paths;
  int cost;

  CBSNode(): cost(0) {}

  // this constructor helps to generate child nodes
  CBSNode(const CBSNode& parent):
    constraints(parent.constraints), paths(parent.paths), cost(0) {}
};

struct Collision
{
  Path path_a;
  Path path_b;
  int time;
  int location;
  int idx_a;
  int idx_b;
  int type;

  Collision():time(0){}
  Collision(Path a, Path b, int t, int l, int ia, int ib, int type): 
    path_a(a), path_b(b), time(t), location(l), idx_a(ia), idx_b(ib), type(type){}
};

// This function is used by priority_queue to prioritize CBS nodes
struct CompareCBSNode {
  bool operator()(const CBSNode* n1, const CBSNode* n2) {
    return n1->cost > n2->cost; // prefer smaller cost
  }
};

class CBS {
public:
  Collision findCollision(CBSNode* node, Path a, int agent_index);
  void handleCollision(priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> * open,
                        CBSNode* parentNode, Collision collision);
  int getSumOfCosts(CBSNode* node);
  vector<Path> find_solution();
  explicit CBS(const MAPFInstance& ins): a_star(ins) {}
  ~CBS();


private:
  AStarPlanner a_star;

  // all_nodes stores the pointers to CBS nodes
  // so that we can release the memory properly when
  // calling the destructor ~CBS()
  list<CBSNode*> all_nodes;

};



/*
* Does CBS always terminate in finite time when the input MAPF instance is unsolvable? If not, provide an example 
* MAPF instance. If so, provide a proof.
*
* This does not always terminate in finite time. It's far too complex!
* Reasoning:
* Every vertex, V, can in the graph that is not blocked can have a constraint associated with an (or all) agent(s),
* n, for an arbitrarily long period of time, t, * 2 for each branch. I'll ballpark this at O(2^(2*n*V*t)). Given
* each of these variables grow proportionally to each other, with n likely being the smallest, this is approximately
* O(2^x^3)! Moreover, this evergrowing list of constraints can become large fast, and must be referenced for each 
* branch of a collision, for each step of the internal a* algorithm, say  O(2*|V|*|E|*log*|V|) 
* 
*/