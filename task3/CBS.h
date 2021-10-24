#pragma once
#include "AStarPlanner.h"
#include <tuple>

struct CBSNode {
    list<Constraint> constraints;
    vector<Path> paths;
    int cost;

    CBSNode(): cost(0) {}

    // this constructor helps to generate child nodes
    CBSNode(const CBSNode& parent):
            constraints(parent.constraints), paths(parent.paths), cost(0) {}
};

// This function is used by priority_queue to prioritize CBS nodes
struct CompareCBSNode {
    bool operator()(const CBSNode* n1, const CBSNode* n2) {
        return n1->cost > n2->cost; // prefer smaller cost
    }
};

class CBS {
public:
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
