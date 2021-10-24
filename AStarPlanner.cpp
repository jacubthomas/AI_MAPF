#include "AStarPlanner.h"
#include <queue>
#include <unordered_map>
#include <algorithm> // reverse

ostream& operator<<(ostream& os, const Path& path)
{
    for (auto loc : path) {
        os << loc << " ";
    }
    return os;
}

Path AStarPlanner::make_path(const AStarNode* goal_node) const {
    Path path;
    const AStarNode* curr = goal_node;
    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
    return path;
}

Path AStarPlanner::find_path(int agent_id, const list<Constraint>& constraints) {
  // TODO Task 1 implement space_time A*
    int start_location = ins.start_locations[agent_id];
    int goal_location = ins.goal_locations[agent_id];

    // Open list
    priority_queue<AStarNode*, vector<AStarNode*>, CompareAStarNode> open;

    // Unordered map is an associative container that contains key-value pairs with unique keys.
    // The following unordered map is used for duplicate detection, where the key is the location of the node.
    unordered_map<int, AStarNode*> all_nodes;
    // TODO: For Task 1, you need to replace the above line with
    //  unordered_map<pair<int, int>, AStarNode*, hash_pair> all_nodes;

    int h = ins.get_Manhattan_distance(start_location, goal_location); // h value for the root node
    auto root = new AStarNode(start_location, 0, h, nullptr);
    open.push(root);

    Path path;
    while (!open.empty()) {
        auto curr = open.top();
        open.pop();

        // goal test
        if (curr->location == goal_location) {
            path = make_path(curr);
            break;
        }

        // generate child nodes
        for (auto next_location : ins.get_adjacent_locations(curr->location)) {
            auto it = all_nodes.find(next_location);
            if (it == all_nodes.end()) {// the location has not been visited before
                int next_g = curr->g + 1;
                int next_h = ins.get_Manhattan_distance(next_location, goal_location);
                auto next = new AStarNode(next_location, next_g, next_h, curr);
                open.push(next);
                all_nodes[next_location] = next;
            }
            // Note that if the location has been visited before,
            // next_g + next_h must be greater than or equal to the f value of the existing node,
            // because we are searching on a 4-neighbor grid with uniform-cost edges.
            // So we don't need to update the existing node.
        }
    }

    // release memory
    for (auto n : all_nodes)
        delete n.second;

    return path;
}
