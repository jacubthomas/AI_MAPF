#include "MAPFInstance.h"
#include <fstream>
#include <iostream>

bool MAPFInstance::load_instance(const string& fname) {
    ifstream myfile (fname.c_str(), ios_base::in);
    if (myfile.is_open()) {
        myfile >> rows >> cols; // read the size of the map
        my_map.resize(rows * cols);

        // read map
        char c;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                myfile >> c;
                my_map[cols * i + j] = (c != '.'); // the cell is unblocked iff it is '.'
            }
        }

        myfile >> num_of_agents; // read the number of agents
        start_locations.resize(num_of_agents);
        goal_locations.resize(num_of_agents);

        // read the start and goal locations
        int start_x, start_y, goal_x, goal_y;
        for (int i = 0; i < num_of_agents; i++) {
            myfile >> start_x >> start_y >> goal_x >> goal_y;
            start_locations[i] = linearize_coordinate(start_x, start_y);
            goal_locations[i] = linearize_coordinate(goal_x, goal_y);
        }
        myfile.close();

        // initialize moves_offset array
        moves_offset[valid_moves_t::NORTH] = -cols;
        moves_offset[valid_moves_t::EAST] = 1;
        moves_offset[valid_moves_t::SOUTH] = cols;
        moves_offset[valid_moves_t::WEST] = -1;
        return true;
    } else
        return false;
}

void MAPFInstance::print_instance() const {
    cout << "Map:" << endl;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (my_map[cols * i + j])
                cout << "@"; // obstacle
            else
                cout << "."; // free cell
        }
        cout << endl;
    }
    cout << num_of_agents << " agents:" << endl;
    for (int i = 0; i < num_of_agents; i++) {
        cout << "a" << i << ": " << start_locations[i] << "-->" << goal_locations[i] << endl;
    }
}

int MAPFInstance::get_Manhattan_distance(int from, int to) const {
    int from_x = row_coordinate(from);
    int from_y = col_coordinate(from);
    int to_x = row_coordinate(to);
    int to_y = col_coordinate(to);
    return abs(from_x - to_x) + abs(from_y - to_y);
}

list<int> MAPFInstance::get_adjacent_locations(int location) const {
	list<int> locations;
	for (int direction : moves_offset) {
		int next_location = location + direction;
		if (0 <= next_location && next_location < cols * rows && // next_location is on the map
		    !my_map[next_location] && // next_location is not blocked
		    get_Manhattan_distance(location, next_location) <= 1) // it indeed moves to a neighbor location
            locations.push_back(next_location);
	}
	return locations;
}
