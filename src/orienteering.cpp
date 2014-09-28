// ============================================================================
//
//    Copyright(c)  2014 Zhan Haoxun.
//
//       Filename:  orienteering.cpp
//
//    Description:  Worksapp Examination 1
//
//        Version:  1.0
//        Created:  09/24/2014 10:04:09
//       Compiler:  g++-4.8.2
//
//         Author:  Zhan Haoxun (huntzhan), programmer.zhx@gmail.com
//
// ============================================================================


// ============================================================================
// # Introduction.
// ============================================================================


// ============================================================================
// Related standard libraries.
// ============================================================================
#include <algorithm>
#include <bitset>
#include <cstddef>
#include <iostream>
#include <istream>
#include <iterator>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <set>
#include <utility>
#include <vector>


using std::bitset;
using std::cin;
using std::cout;
using std::distance;
using std::endl;
using std::fill;
using std::find;
using std::istream;
using std::map;
using std::min;
using std::numeric_limits;
using std::pair;
using std::queue;
using std::set;
using std::string;
using std::size_t;
using std::unordered_map;
using std::vector;


// ============================================================================
// Global type alias.
// ============================================================================
// Two-dimension matrix represents shortest path of every pair of targets.
using DistanceMatrix = vector<vector<int>>;
// Coordinate of symbols. Given that "Coordinate example;", "example.first"
// represents the row index and "example.second" represents the column index.
using Coordinate = pair<int, int>;
// Types used in TSPCalculator.
using StrictBitset = bitset<32>;
using GroupIndexValueMapping = map<size_t, int>;
using GroupValue = unordered_map<StrictBitset, GroupIndexValueMapping>;


// ============================================================================
// Global variables.
// ============================================================================
const char kStartSymbol = 'S';
const char kGoalSymbol = 'G';
const char kCheckpointSymbol = '@';
const char kCloseBlockSymbol = '#';
const int kIntMax = numeric_limits<int>::max();


// ============================================================================
// Declearation of classes.
// ============================================================================
class InputHandler {
 public:
  void ReadFromInputStream(istream *in_ptr);

  vector<string> orienteering_map_;
  Coordinate start_;
  Coordinate goal_;
  vector<Coordinate> checkpoints_;

 private:
  void RecordCoordinate(const size_t &row_index, const size_t &column_index);
};


class DistanceMatrixGeneratorInterface {
 public:
  // interface.
  virtual bool Generate(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets) = 0;
  // output.
  DistanceMatrix distance_matrix_;

  // Shared logic.
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

  void BuildNeighbors(const vector<string> &orienteering_map);
  // valid neighbors of each coordinate.
  map<Coordinate, vector<Coordinate>> neighbors_;
};


class DMGeneratorWithBFS : public DistanceMatrixGeneratorInterface {
 public:
  bool Generate(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets) override;

 private:
  bool FindShortestPaths(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets,
      const size_t &source_index);
};


class TSPCalculator {
 public:
  int CalculateMinLength(const DistanceMatrix &distance_matrix);

 private:
  void UpdateMinLengths(
      const StrictBitset &checkpoint_set,
      const size_t &checkpoint_size,
      const DistanceMatrix &distance_matrix,
      GroupValue *min_lengths_ptr);
};


// ============================================================================
// Definition of classes.
// ============================================================================
void InputHandler::RecordCoordinate(const size_t &row_index,
                                    const size_t &column_index) {
  const char &symbol = orienteering_map_[row_index][column_index];
  switch (symbol) {
    case kStartSymbol: {
      start_ = Coordinate(row_index, column_index);
      break;
    }
    case kGoalSymbol: {
      goal_ = Coordinate(row_index, column_index);
      break;
    }
    case kCheckpointSymbol: {
      checkpoints_.push_back(Coordinate(row_index, column_index));
      break;
    }
  }
}


// Load orienteering map from standard input.
void InputHandler::ReadFromInputStream(istream *in_ptr) {
  // setup orienteering_map_.
  istream &in = *in_ptr;
  size_t width = 0, height = 0;
  in >> width >> height;

  for (size_t counter = 0; counter != height; ++counter) {
    string line;
    in >> line;
    orienteering_map_.push_back(line);
  }

  // find start, goal and checkpoints.
  for (size_t row_index = 0; row_index != height; ++row_index) {
    for (size_t column_index = 0; column_index != width; ++column_index) {
      RecordCoordinate(row_index, column_index);
    }
  }
}


inline bool DistanceMatrixGeneratorInterface::IsValid(
    const Coordinate &coordinate,
    const int &row_size, const int &column_size) {
  const int &x = coordinate.first;
  const int &y = coordinate.second;
  return (0 <= x && x < row_size) && (0 <= y && y < column_size);
}


vector<Coordinate> DistanceMatrixGeneratorInterface::NextCoordinates(
    const Coordinate &coordinate,
    const int &row_size, const int &column_size) {
  // indexs.
  const int &x = coordinate.first;
  const int &y = coordinate.second;
  // generate coordinates adjacent to current coordinates.
  vector<Coordinate> next_coordinates;
  for (auto &next_coordinate : {Coordinate(x - 1, y),     // up.
                                Coordinate(x + 1, y),     // down.
                                Coordinate(x, y - 1),     // left.
                                Coordinate(x, y + 1)}) {  // right.
    if (IsValid(next_coordinate, row_size, column_size)) {
      next_coordinates.push_back(next_coordinate);
    }
  }
  return next_coordinates;
}


// Generate valid neighbors for each coordinate.
void DistanceMatrixGeneratorInterface::BuildNeighbors(
    const vector<string> &orienteering_map) {

  const size_t row_size = orienteering_map.size();
  const size_t column_size = orienteering_map.front().size();
  auto IsCloseBlock = [&](const size_t &x, const size_t &y) {
    return orienteering_map[x][y] == kCloseBlockSymbol;
  };

  for (size_t row_index = 0; row_index != row_size; ++row_index) {
    for (size_t column_index = 0;
         column_index != column_size; ++column_index) {
      // Don't process close block.
      if (IsCloseBlock(row_index, column_index)) { continue; }

      Coordinate current(row_index, column_index);
      auto neighbors = NextCoordinates(current, row_size, column_size);
      // filter out close block.
      auto neighbor_iter = neighbors.begin();
      while (neighbor_iter != neighbors.end()) {
        const int &x = neighbor_iter->first;
        const int &y = neighbor_iter->second;
        if (IsCloseBlock(x, y)) {
          neighbor_iter = neighbors.erase(neighbor_iter);
        } else {
          ++neighbor_iter;
        }
      }
      // keep neighbors.
      neighbors_[current] = neighbors;
    }
  }
}


template <typename Element>
vector<vector<Element>> DistanceMatrixGeneratorInterface::InitMatrix(
    const int &row_size, const int &column_size,
    const Element &default_value) {
  vector<vector<Element>> matrix;
  for (int counter = 0; counter != row_size; ++counter) {
    // init a row for each target.
    matrix.push_back(vector<Element>(column_size, default_value));
  }
  return matrix;
}


// Find the shortest paths from single source to all the others by using BFS.
bool DMGeneratorWithBFS::FindShortestPaths(
    const vector<string> &orienteering_map,
    const vector<Coordinate> &targets,
    const size_t &source_index) {
  set<Coordinate> targets_set(targets.cbegin(), targets.cend());
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
  size_t searched_targets = 0;

  // binds out_queue_ptr to first_queue, binds in_queue_ptr to second_queue.
  auto out_queue_ptr = &first_queue;
  auto in_queue_ptr = &second_queue;
  // carry out BFS.
  while (!out_queue_ptr->empty()
         && searched_targets < targets.size()) {
    const auto &coordinate = out_queue_ptr->front();
    // check target in constant time.
    if (targets_set.find(coordinate) != targets_set.end()) {
      // current coordinate is target.
      // get index of target in targets, with the same coordinate.
      const size_t target_index = distance(
          targets.cbegin(),
          find(targets.cbegin(), targets.cend(), coordinate));
      // now, we can fill element of the distance_matrix_, pointed by
      // (source_index, target_index), with the value distance.
      distance_matrix_[source_index][target_index] = current_distance;
      ++searched_targets;
    }

    // generate coordinates for searching.
    for (const auto &next_coordinate : neighbors_[coordinate]) {
      const int &x = next_coordinate.first;
      const int &y = next_coordinate.second;
      // searched.
      if (searched_coordinate[x][y]) { continue; }

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


bool DMGeneratorWithBFS::Generate(
    const vector<string> &orienteering_map,
    const vector<Coordinate> &targets) {

  // init distance_matrix_.
  const int target_size = targets.size();
  distance_matrix_ = InitMatrix(target_size, target_size, 0);
  BuildNeighbors(orienteering_map);

  for (size_t source_index = 0;
       source_index != targets.size(); ++source_index) {
    bool is_success = FindShortestPaths(
        orienteering_map, targets, source_index);
    if (!is_success) {
      return false;
    }
  }
  // all is well.
  return true;
}


void TSPCalculator::UpdateMinLengths(
    const StrictBitset &checkpoint_set,
    const size_t &checkpoint_size,
    const DistanceMatrix &distance_matrix,
    GroupValue *min_lengths_ptr) {

  GroupValue &min_lengths = *min_lengths_ptr;

  vector<size_t> indices;
  for (size_t index = 0; index != checkpoint_size; ++index) {
    if (checkpoint_set.test(index)) {
      indices.push_back(index);
    }
  }

  for (const size_t &index : indices) {
    // make previous subset.
    StrictBitset previous_subset(checkpoint_set);
    previous_subset.reset(index);

    // init total_minimum to max value.
    int local_minimum = kIntMax;
    // loop over the rest of indices.
    for (const size_t &other_index : indices) {
      if (other_index == index) { continue; }
      // access previous result.
      const int &previous_length = min_lengths[previous_subset][other_index];
      local_minimum = min(
          local_minimum,
          previous_length + distance_matrix[index][other_index]);
    }                        
    // update min_lengths.
    min_lengths[checkpoint_set][index] = local_minimum;
  }
}


// Solve TSP problem based on dynamic programming.
int TSPCalculator::CalculateMinLength(const DistanceMatrix &distance_matrix) {
  const int dimension = distance_matrix.size();
  const size_t goal_index = dimension - 1;
  const size_t source_index = dimension - 2;
  const size_t checkpoint_size = dimension - 2;

  // for the case that there's no checkpoints.
  if (checkpoint_size == 0) {
    return distance_matrix[source_index][goal_index];
  }

  // checkpoint_sets for DP.
  vector<vector<StrictBitset>> checkpoint_sets(checkpoint_size + 1);
  unsigned end_of_bit_pattern = 1 << checkpoint_size;
  for (unsigned bit_pattern = 1;
       bit_pattern < end_of_bit_pattern; ++bit_pattern) {
    StrictBitset checkpoint_set(bit_pattern);
    auto &bucket = checkpoint_sets[checkpoint_set.count()];
    bucket.push_back(checkpoint_set);
  }

  // init.
  GroupValue min_lengths;
  const auto &init_sets = checkpoint_sets[1];
  for (size_t checkpoint_index = 0;
       checkpoint_index != checkpoint_size; ++checkpoint_index) {
    const auto &init_set = init_sets[checkpoint_index];
    min_lengths[init_set][checkpoint_index] =
        distance_matrix[source_index][checkpoint_index];
  }
  // internal step.
  for (size_t subset_size = 2;
       subset_size <= checkpoint_size; ++subset_size) {
    for (const auto &checkpoint_set : checkpoint_sets[subset_size]) {
      UpdateMinLengths(checkpoint_set, checkpoint_size,
                       distance_matrix, &min_lengths);
    }
  }
  // final step.
  const auto &all_checkpoints = checkpoint_sets[checkpoint_size].front();
  int total_minimum = kIntMax;
  for (size_t checkpoint_index = 0;
       checkpoint_index != checkpoint_size; ++checkpoint_index) {
    total_minimum= min(
        total_minimum,
        min_lengths[all_checkpoints][checkpoint_index]
        + distance_matrix[goal_index][checkpoint_index]);
  }
  return total_minimum;
}


// ============================================================================
// Skeleton code for the examination.
// ============================================================================
class Orienteering {
 public:
  void main();
};


void Orienteering::main() {
  InputHandler input_handler;
  input_handler.ReadFromInputStream(&cin);

  auto targets = input_handler.checkpoints_;
  targets.push_back(input_handler.start_);
  targets.push_back(input_handler.goal_);
  const auto &orienteering_map = input_handler.orienteering_map_;

  DMGeneratorWithBFS generator;
  bool flag = generator.Generate(orienteering_map, targets);
  if (!flag) {
    cout << -1 << endl;
    return;
  }

  const auto &distance_matrix = generator.distance_matrix_;
  TSPCalculator calculator;
  int min_length = calculator.CalculateMinLength(distance_matrix);

  // output.
  cout << min_length << endl;
}


// int main(int argc, char* argv[]) {
//   Orienteering o;
//   o.main();
//   return 0;
// }
