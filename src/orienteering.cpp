// ============================================================================
//
//       Filename:  orienteering.cpp
//
//    Description:  
//
//        Version:  1.0
//        Created:  09/24/2014 10:04:09
//       Revision:  none
//       Compiler:  g++
//
//         Author:  Zhan Haoxun (huntzhan), programmer.zhx@gmail.com
//   Organization:  
//
// ============================================================================

#include <istream>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <unordered_set>
#include <cstddef>
#include <functional>
#include <queue>
#include <algorithm>
#include <iterator>


using std::istream;
using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::pair;
using std::make_pair;
using std::string;
using std::unordered_set;
using std::size_t;
using std::queue;
using std::find;
using std::distance;

// Two-dimension matrix represents shortest path of every pair of targets.
using DistanceMatrix = vector<vector<int>>;
// Coordinate of symbols. Given that "Coordinate example", "example.first"
// represents the row index and "example.second" represents the column index.
using Coordinate = pair<int, int>;
// hash for Coordinate.
namespace std {

template <>
struct hash<Coordinate> {
  using result_type = size_t;
  using argument_type = Coordinate;
  // call operator.
  size_t operator()(const Coordinate &target) {
    // XOR of hash results.
    return hash<int>()(target.first) ^ hash<int>()(target.second);
  }
};

}  // namespace std


const char kStartSymbol = 'S';
const char kGoalSymbol = 'G';
const char kCheckpointSymbol = '@';
const char kOpenBlockSymbol = '.';
const char kCloseBlockSymbol = '#';


// Strategy:
// 1. Find the shortest path between each pairs of targets(start, goal,
// checkpoint), based on BFS.
// 2. The problem were transformed to be a TSP problem. Solve the problem with 
// dynamic programming.


class InputHandler {
 public:
  void ReadFromInputStream(istream *in_ptr);

  vector<string> orienteering_map_;
  Coordinate start_;
  Coordinate goal_;
  vector<Coordinate> checkpoints_;

 private:
  void RecordCoordinate(const int &row_index, const int &column_index);
};

void InputHandler::RecordCoordinate(const int &row_index,
                                    const int &column_index) {
  const char &symbol = orienteering_map_[row_index][column_index];
  switch (symbol) {
    case kStartSymbol: {
      start_ = make_pair(row_index, column_index);
      break;
    }
    case kGoalSymbol: {
      goal_ = make_pair(row_index, column_index);
      break;
    }
    case kCheckpointSymbol: {
      checkpoints_.push_back(
          make_pair(row_index, column_index));
      break;
    }
  }
}

// Load orienteering map from standard input.
void InputHandler::ReadFromInputStream(istream *in_ptr) {
  // setup orienteering_map_.
  int width = 0, height = 0;
  *in_ptr >> width >> height;

  for (int counter = 0; counter != height; ++counter) {
    string line;
    *in_ptr >> line;
    orienteering_map_.push_back(line);
  }

  // find start, goal and checkpoints.
  for (int row_index = 0; row_index != height; ++row_index) {
    for (int column_index = 0; column_index != width; ++column_index) {
      RecordCoordinate(row_index, column_index);
    }
  }
}


class DistanceMatrixGenerator {
 public:
  bool Generate(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets);

  DistanceMatrix distance_matrix;

 private:
  bool FindShortestPathFromSingleSource(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets,
      const int &source_index);

  template <typename Element>
  vector<vector<Element>> InitMatrix(
      const int &row_size, const int &column_size,
      const Element &default_value);

  bool IsValid(
      const Coordinate &coordinate,
      const int &row_size, const int &column_size);

  vector<Coordinate> NextCoordinates(
      const Coordinate &coordinate,
      const int &row_size, const int &column_size);
};

template <typename Element>
vector<vector<Element>> DistanceMatrixGenerator::InitMatrix(
    const int &row_size, const int &column_size,
    const Element &default_value) {
  vector<vector<Element>> matrix;
  for (int counter = 0; counter != row_size; ++counter) {
    // init a row for each target.
    matrix.push_back(vector<Element>(column_size, default_value));
  }
  return matrix;
}

bool DistanceMatrixGenerator::IsValid(
    const Coordinate &coordinate,
    const int &row_size, const int &column_size) {
  const int &x = coordinate.first;
  const int &y = coordinate.second;
  return (0 <= x && x < row_size) && (0 <= y && y < column_size);
}

vector<Coordinate> DistanceMatrixGenerator::NextCoordinates(
    const Coordinate &coordinate,
    const int &row_size, const int &column_size) {
  // indexs.
  const int &x = coordinate.first;
  const int &y = coordinate.second;
  // generate coordinates adjacent to current coordinates.
  vector<Coordinate> next_coordinates;
  for (const auto &next_coordinate : {make_pair(x - 1, y),
                                      make_pair(x + 1, y),
                                      make_pair(x, y - 1),
                                      make_pair(x, y + 1)}) {
    if (IsValid(next_coordinate, row_size, column_size)) {
      next_coordinates.push_back(
          std::move(next_coordinate));
    }
  }
  return next_coordinates;
}

bool DistanceMatrixGenerator::Generate(
    const vector<string> &orienteering_map,
    const vector<Coordinate> &targets) {
  // init distance_matrix.
  const int target_size = targets.size();
  distance_matrix = InitMatrix(target_size, target_size, 0);
  // fill the matrix.
  for (int source_index = 0; source_index != targets.size(); ++source_index) {
    bool success_flag = FindShortestPathFromSingleSource(
        orienteering_map, targets, source_index);
    if (!success_flag) { return false; }
  }
  // all is well.
  return true;
}

// Find the shortest paths from single source to all the others by using BFS.
bool DistanceMatrixGenerator::FindShortestPathFromSingleSource(
    const vector<string> &orienteering_map,
    const vector<Coordinate> &targets,
    const int &source_index) {
  // speed up search by using hash table.
  unordered_set<Coordinate> hashed_targets(targets.cbegin(), targets.cend());
  // get source coordinate.
  const Coordinate &source_coordinate = targets[source_index];

  // init first_queue with first target; second_queue is empty.
  queue<Coordinate> first_queue, second_queue;
  first_queue.push(source_coordinate);

  // init matrix that records searched coordinates.
  const int row_size = orienteering_map.size();
  const int column_size = orienteering_map.front().size();
  auto searched_coordinate = InitMatrix(row_size, column_size, false);
  // mark source coordinate as searched.
  const int &x = source_coordinate.first;
  const int &y = source_coordinate.second;
  searched_coordinate[x][y] = true;

  // counter of distance.
  int current_distance = 0;
  // counter of searched targets. 
  int searched_targets = 0;

  // binds out_queue_ptr to first_queue, binds in_queue_ptr to second_queue.
  auto out_queue_ptr = &first_queue;
  auto in_queue_ptr = &second_queue;
  // carry out BFS.
  while (!out_queue_ptr->empty()
         && searched_targets < targets.size()) {
    const auto &coordinate = out_queue_ptr->front();
    // check target in constant time.
    if (hashed_targets.find(coordinate) != hashed_targets.end()) {
      // current coordinate is target.
      // get index of target in targets, with the same coordinate.
      const int target_index = distance(
          targets.cbegin(),
          find(targets.cbegin(), targets.cend(), coordinate));
      // now, we can fill element of the distance_matrix, pointed by
      // (source_index, target_index), with the value distance.
      distance_matrix[source_index][target_index] = current_distance;
      ++searched_targets;
    }

    // generate coordinates for searching.
    auto next_coordinates = NextCoordinates(
        coordinate, row_size, column_size);
    for (const Coordinate &next_coordinate : next_coordinates) {
      const int &x = next_coordinate.first;
      const int &y = next_coordinate.second;
      // searched.
      if (searched_coordinate[x][y]) { continue; }
      // close block.
      if (orienteering_map[x][y] == kCloseBlockSymbol) { continue; }

      // valid coordinate!
      in_queue_ptr->push(next_coordinate);
      // mark visited.
      searched_coordinate[x][y] = true;
    }
    // finish processing current coordinate, pop it.
    out_queue_ptr->pop();

    // update current_distance.
    if (out_queue_ptr->empty()) {
      // out_queue is empty means ALL coordinates with the same distances have
      // been searched.
      std::swap(out_queue_ptr, in_queue_ptr);
      ++current_distance;
    }
  }

  if (searched_targets == targets.size()) {
    return true;
  } else {
    return false;
  }
}



// Skeleton code for the examination.
// class Orienteering {
//  public:
//   void main();
// };
// 
// void Orienteering::main() {
//   InputHandler input_handler;
//   input_handler.ReadFromInputStream(&cin);
// }
//
// int main(int argc, char* argv[]) {
//   Orienteering o;
//   o.main();
//   return 0;
// }
