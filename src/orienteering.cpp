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
// 
// The code is implemented with features as follow:
//  
// * Follows *Google C++ Style Guide*.
// * Make good usage of C++11 features.
// * Highly focus on the readablity and performance.
// 
// The following content are seperated into six sessions:
// 
// 1. Related standard libraries
// 2. Global type alias.
// 3. Global variables.
// 4. Declearation of classes.
// 5. Definition of classes.
// 6. Skeleton code for the examination.
//
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
#include <set>
#include <string>
#include <unordered_map>
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
using std::size_t;
using std::string;
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
// Load the orienteering map from input stream. Sample usage:
//     InputHandler input_handler;
//     input_handler.ReadFromInputStream(&cin);  // Load from cin.
//
//     // Loaded orienteering map.
//     const auto &orienteering_map = input_handler.orienteering_map_;
//   
//     // Coordinates that useful for subsequently processing.
//     const auto &targets = input_handler.checkpoints_;
//     const auto &start = input_handler.start_;
//     const auto &goal = input_handler.goal_;
//
class InputHandler {
 public:
  // Accept a pointer to input stream, then load the orienteering map from such
  // stream.
  void ReadFromInputStream(istream *in_ptr);

  // Output.
  vector<string> orienteering_map_;
  Coordinate start_;
  Coordinate goal_;
  vector<Coordinate> checkpoints_;

 private:
  // Auxiliary function that records the coordinates of start, goal and
  // checkpoints.
  void RecordCoordinate(const size_t &row_index, const size_t &column_index);
};


// Defines the interface of distance matrix generator. Derived class should
// override `Generate` member function, and fill the `distance_matrix_` data
// member in the overrided `Generate` member function. `Generate` return true
// if everything is fine. Otherwise the function would return false,
// indicating "players cannot arrive at the goal from the start by passing all
// the checkpoints".
//
// The class also provoides some useful operations during the generation of
// distance matrix. `InitMatrix` would generator a `row_size` * `column_size`
// matrix filled with `default_value`. `BuildNeighbors` could build the
// mapping from each coordinate to its valid neighbors, in which the valid
// neighbor means neither the neighbor is close block nor is out of range.
//
class DistanceMatrixGeneratorInterface {
 public:
  // Interface should be overrided by derived class.
  virtual bool Generate(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets) = 0;

  // Genenrate a `row_size` * `column_size` with `default_value`.
  template <typename Element>
  vector<vector<Element>> InitMatrix(
      const int &row_size, const int &column_size,
      const Element &default_value);
  // Build the all valid neighbors in orienteering map, the result would be
  // saved in `distance_matrix_` data member.
  void BuildNeighbors(const vector<string> &orienteering_map);

  // Valid neighbors of each coordinate.
  map<Coordinate, vector<Coordinate>> neighbors_;
  // Output.
  DistanceMatrix distance_matrix_;

 private:
  // Auxiliary function genenrates coordinates of neighbors that are not out
  // of range.
  vector<Coordinate> NextCoordinates(
      const Coordinate &coordinate,
      const int &row_size, const int &column_size);
};


// Building the distance by using BFS(breadth-first-search). Briefly,
// `Generate` calls `FindShortestPaths` to genenrate distance matrix.
// `FindShortestPaths` would carry out the distances from single source to
// all the other goals.
//
// Sample usage:
//     DMGeneratorWithBFS generator;
//     bool flag = generator.Generate(orienteering_map, targets);
//     if (!flag) {
//       // Targets are not connected.
//     }
//     // Access the output.
//     const auto &distance_matrix = generator.distance_matrix_;
//
class DMGeneratorWithBFS : public DistanceMatrixGeneratorInterface {
 public:
  bool Generate(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets) override;

 private:
  // Calculate the distances from a single source to multiple goals. The
  // single source is in `targets` and indexed by `source_index`, while the
  // goals are the elements of `targets` except the one indexed by
  // `source_index`.
  //
  // Return true if the function can carry out all the distances fron source
  // to goals, in other word, all the `targets` are connected. Once again,
  // the results of calculating would be kept in `distance_matrix_`.
  //
  bool FindShortestPaths(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets,
      const size_t &source_index);
};


// Defines the interface of calculating "the minimum distance from the start
// to the goal with passing all the checkpoints". Since this is basically a TSP
// problem, the interface is named `TSPCalculatorInterface`. Derived class
// should override `CalculateMinLength` member function.
//
class TSPCalculatorInterface {
 public:
  // Interface should be overrided by derived class.
  //
  // The input of `CalculateMinLength` is a little bit tricky. Suppose
  // `distance_matrix` is a N * N 2-D array, the function would consider
  // (N - 2) as the index of start point, (N - 1) as the index of goal point
  // (the index starts in 0). If there's no checkpoints, the `distance_matrix`
  // would be a 2 * 2 2-D array, with index 0 points to start, index 1 points
  // to goal.
  //
  virtual int CalculateMinLength(const DistanceMatrix &distance_matrix) = 0;
};


// Solve the TSP problem based on dynamic programming. The time complexity is
// O(n^2 * 2^n). `TSPCalculatorWithBitset` is implemented with std::bitset,
// with the purpose of enhancing readability of the code. But with the
// consideration of performance, another version of the same strategy is
// implemented with built-in arithmatic type, which is
// `TSPCalculatorWithBitOperation`. Since `TSPCalculatorWithBitOperation`
// outperforms `TSPCalculatorWithBitset`, the TSP problem would be handled by
// `TSPCalculatorWithBitOperation`.
//
// Sample usage:
//     TSPCalculatorWithBitset calculator;
//     int min_length = calculator.CalculateMinLength(distance_matrix);
//
class TSPCalculatorWithBitset : public TSPCalculatorInterface {
  // Auxiliary types for coding.
  using StrictBitset = bitset<32>;
  using GroupIndexValueMapping = map<size_t, int>;
  using GroupValue = unordered_map<StrictBitset, GroupIndexValueMapping>;

 public:
  int CalculateMinLength(const DistanceMatrix &distance_matrix) override;

 private:
  void UpdateMinLengths(
      const StrictBitset &checkpoint_set,
      const size_t &checkpoint_size,
      const DistanceMatrix &distance_matrix,
      GroupValue *min_lengths_ptr);
};


// Solve the TSP problem based on dynamic programming. This class implemented
// the same strategy as `TSPCalculatorWithBitset`, but in a more effcient
// and less readable way.
//
// Sample usage:
//     TSPCalculatorWithBitOperation calculator;
//     int min_length = calculator.CalculateMinLength(distance_matrix);
//
class TSPCalculatorWithBitOperation : public TSPCalculatorInterface {
 public:
  int CalculateMinLength(const DistanceMatrix &distance_matrix) override;
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


vector<Coordinate> DistanceMatrixGeneratorInterface::NextCoordinates(
    const Coordinate &coordinate,
    const int &row_size, const int &column_size) {
  // Check if the coordinate is out of range.
  auto IsValid = [&](const Coordinate &coordinate) -> bool {
    const int &x = coordinate.first;
    const int &y = coordinate.second;
    return (0 <= x && x < row_size) && (0 <= y && y < column_size);
  };
  // indexs.
  const int &x = coordinate.first;
  const int &y = coordinate.second;
  // generate coordinates adjacent to current coordinates.
  vector<Coordinate> next_coordinates;
  for (auto &next_coordinate : {Coordinate(x - 1, y),     // up.
                                Coordinate(x + 1, y),     // down.
                                Coordinate(x, y - 1),     // left.
                                Coordinate(x, y + 1)}) {  // right.
    if (IsValid(next_coordinate)) {
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
      if (IsCloseBlock(row_index, column_index)) {
        // Don't process close block.
        continue;
      }
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
  using Line = vector<Element>;
  return vector<Line>(
      row_size,
      Line(column_size, default_value));
}


// Find the shortest paths from single source to all the others by using BFS.
bool DMGeneratorWithBFS::FindShortestPaths(
    const vector<string> &orienteering_map,
    const vector<Coordinate> &targets,
    const size_t &source_index) {
  // speedup query by using std::set.
  set<Coordinate> targets_set(targets.cbegin(), targets.cend());
  // get source coordinate.
  const Coordinate &source_coordinate = targets[source_index];

  // init first_queue with first target; second_queue is empty.
  queue<Coordinate> first_queue, second_queue;
  first_queue.push(source_coordinate);
  // binds out_queue_ptr to first_queue, binds in_queue_ptr to second_queue.
  auto out_queue_ptr = &first_queue;
  auto in_queue_ptr = &second_queue;

  // init matrix that records searched coordinates.
  const int row_size = orienteering_map.size();
  const int column_size = orienteering_map.front().size();
  auto searched_coordinate = InitMatrix(row_size, column_size, false);
  // mark source coordinate as searched.
  const int &current_x = source_coordinate.first;
  const int &current_y = source_coordinate.second;
  searched_coordinate[current_x][current_y] = true;

  // counter of distance.
  int current_distance = 0;
  // counter of searched targets.
  size_t searched_targets = 0;

  // carry out BFS.
  while (!out_queue_ptr->empty()
         && searched_targets < targets.size()) {
    // get the first element from queue.
    const auto &current = out_queue_ptr->front();

    if (targets_set.find(current) != targets_set.end()) {
      // `current` is a target.
      // get index of `target` in `targets`, with the same coordinate.
      const size_t target_index = distance(
          targets.cbegin(),
          find(targets.cbegin(), targets.cend(), current));
      // now, we can fill element of the distance_matrix_, pointed by
      // (source_index, target_index), with the value of `current_distance`.
      distance_matrix_[source_index][target_index] = current_distance;
      ++searched_targets;
    }

    // generate coordinates for searching.
    for (const auto &neighbor : neighbors_[current]) {
      const int &neighbor_x = neighbor.first;
      const int &neighbor_y = neighbor.second;
      if (searched_coordinate[neighbor_x][neighbor_y]) {
        // searched.
        continue;
      }
      // push neighbor to queue.
      in_queue_ptr->push(neighbor);
      // mark visited.
      searched_coordinate[neighbor_x][neighbor_y] = true;
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


void TSPCalculatorWithBitset::UpdateMinLengths(
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
int TSPCalculatorWithBitset::CalculateMinLength(
    const DistanceMatrix &distance_matrix) {
  const int dimension = distance_matrix.size();
  const size_t goal_index = dimension - 1;
  const size_t source_index = dimension - 2;
  const size_t checkpoint_size = dimension - 2;

  if (checkpoint_size == 0) {
    // for the case that there's no checkpoints.
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


int TSPCalculatorWithBitOperation::CalculateMinLength(
    const DistanceMatrix &distance_matrix) {

  const int dimension = distance_matrix.size();
  const size_t goal_index = dimension - 1;
  const size_t source_index = dimension - 2;
  const size_t checkpoint_size = dimension - 2;
  const unsigned end_of_bit_pattern = 1 << (checkpoint_size + 1);

  if (checkpoint_size == 0) {
    // for the case that there's no checkpoints.
    return distance_matrix[source_index][goal_index];
  }

  // `length_matrix` is a two dimensional matrix, with the first dimension
  // index as the selection of checkpoints and start point, and the second
  // dimension as the index of checkpoint with which the path ends.
  //
  // For instance, suppose the `checkpoint_size` is 3, then we have something
  // like "length_matrix[0xB][2] = 1;". 0xB is equivalent to binary code 1011,
  // the least significant bit 101[1] represents that current set contains the
  // start point. [101]1, three high-order bits, represent the selection of
  // checkpoints, meaning that current set contains checkpoints indexed by 0
  // and 2. "length_matrix[0xB][2]" means current set has a shortest path from
  // start point to the checkpoint indexed by 2, and the value of
  // "length_matrix[0xB][2]" is the length of such shortest path.
  //
  vector<vector<int>> length_matrix(
      end_of_bit_pattern,
      vector<int>(checkpoint_size, kIntMax));

  for (unsigned bit_pattern = 0x3;
       bit_pattern < end_of_bit_pattern; bit_pattern += 0x2) {

    for (unsigned outer_shift = 1, outer_index = 0;
         outer_shift <= checkpoint_size; ++outer_shift, ++outer_index) {
      if (!(bit_pattern & (1 << outer_shift))) {
        // `bit_pattern` do not contains checkpoint indexed by `outer_index`.
        continue;
      }
      // remove checkpoint `outer_index` from `bit_pattern`, create a
      // `subset`.
      unsigned subset = bit_pattern - (1 << outer_shift);
      auto &target = length_matrix[bit_pattern][outer_index];
      if (subset == 0x1) {
        // `subset` contains only the start point, just set the value as the
        // distance from source to checkpoint `outer_index`.
        target = distance_matrix[source_index][outer_index];
        continue;
      }
      for (unsigned inner_shift = 1, inner_index = 0;
           inner_shift <= checkpoint_size; ++inner_shift, ++inner_index) {
        if (bit_pattern & (1 << inner_shift)
            && inner_shift != outer_shift) {
          target = min(
              target,
              length_matrix[subset][inner_index]
              + distance_matrix[outer_index][inner_index]);
        }
      }
    }
  }

  // final step.
  const unsigned all_checkpoints = end_of_bit_pattern - 1;
  int total_minimum = kIntMax;
  for (unsigned checkpoint_index = 0;
       checkpoint_index != checkpoint_size; ++checkpoint_index) {
    total_minimum= min(
        total_minimum,
        length_matrix[all_checkpoints][checkpoint_index]
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
  // TSPCalculatorWithBitset calculator;
  TSPCalculatorWithBitOperation calculator;
  int min_length = calculator.CalculateMinLength(distance_matrix);

  // Output.
  cout << min_length << endl;
}


int main(int argc, char* argv[]) {
  Orienteering o;
  o.main();
  return 0;
}
