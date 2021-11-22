#pragma once
#include "MAPFInstance.h"
#include <ostream>
#include <tuple>

// Vertex constraint <a, x, -1, t, VERTEX>
// that prohibits agent a from being at location x at timestep t
// Edge constraint <a, x, y, t, EDGE>
// that prohibits agent a from moving from locations x to y from timesteps t-1 to t
enum ConstraintType {VERTEX, EDGE};
typedef tuple<int, int, int, int, ConstraintType> Constraint;

// Path is a sequence of locations,
// where path[i] represents the location at timestep i
typedef vector<int> Path;
ostream& operator<<(ostream& os, const Path& path); // used for printing paths

// A hash function used to hash a pair of any kind
// This will be used in Task 1 when you try to
// use pair as the key of an unordered_map
struct hash_pair {
  template <class T1, class T2>
  size_t operator()(const pair<T1, T2>& p) const
  {
    auto hash1 = hash<T1>{}(p.first);
    auto hash2 = hash<T2>{}(p.second);
    return hash1 ^ hash2;
  }
};

struct AStarNode {
  int location;
  int g;
  int h;
  AStarNode* parent;
  int timestep;

  AStarNode(): location(-1), g(-1), h(-1), parent(nullptr), timestep(0) {}
  AStarNode(int location, int g, int h, AStarNode* parent,  int timestep):
    location(location), g(g), h(h), parent(parent),  timestep(timestep) {}
};

// This function is used by priority_queue to prioritize nodes
struct CompareAStarNode {
  bool operator()(const AStarNode* n1, const AStarNode* n2) {
    if (n1->g + n1->h == n2->g + n2->h) // if both nodes have the same f value,
      return n1->h > n2->h; // break ties by preferring smaller h value
    else
      return n1->g + n1->h > n2->g + n2->h; // otherwise, prefer smaller f value
  }
};

class AStarPlanner {
public:
  const MAPFInstance& ins;
  clock_t start;
  double end = 5;

  AStarPlanner(const MAPFInstance& ins): ins(ins) {
    start = clock();
  }
  Path find_path(int agent_id, const list<Constraint>& constraints = {});
  bool evaluateConstraints(Constraint abide1, Constraint abide2, const list<Constraint>& constraints);
  bool isGoalBlocked(int agent_i, int current_location);
private:
  // used to retrieve the path from the goal node
  Path make_path(const AStarNode* goal_node) const;

  void clear();
};

