#pragma once
#include <vector>
#include <list>
#include <string>

using namespace std;

class MAPFInstance {
public:
    vector<int> start_locations;
    vector<int> goal_locations;
    int num_of_agents;

    // return true if the location is blocked by an obstacle.
    inline bool blocked(int location) const {return my_map[location]; }
    inline size_t map_size() const { return rows * cols; }

    // This can be used as admissible heuristics
    int get_Manhattan_distance(int from, int to) const;

    list<int> get_adjacent_locations(int location) const; // return unblocked adjacent locations
    bool load_instance(const string& fname); // load instance from file
    void print_instance() const;

private:
  vector<bool> my_map; // my_map[i] = true iff location i is blocked
  int rows;
  int cols;
  enum valid_moves_t { NORTH, EAST, SOUTH, WEST, MOVE_COUNT };  // MOVE_COUNT is the enum's size
  int moves_offset[4];

  inline int linearize_coordinate(int row, int col) const { return (this->cols * row + col); }
  inline int row_coordinate(int location) const { return location / this->cols; }
  inline int col_coordinate(int location) const { return location % this->cols; }
};
