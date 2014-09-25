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

// Strategy:
// 1. Find the shortest path between each pairs of targets(start, goal,
// checkpoint), based on BFS.
// 2. The problem were transformed to be a TSP problem. Solve the problem with 
// dynamic programming.


#include <algorithm>
#include <bitset>
#include <cstddef>
#include <functional>
#include <future>
#include <iostream>
#include <istream>
#include <iterator>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>


using std::async;
using std::bitset;
using std::cin;
using std::cout;
using std::distance;
using std::endl;
using std::fill;
using std::find;
using std::function;
using std::future;
using std::istream;
using std::next_permutation;
using std::numeric_limits;
using std::pair;
using std::queue;
using std::string;
using std::size_t;
using std::unordered_map;
using std::unordered_set;
using std::vector;

// Two-dimension matrix represents shortest path of every pair of targets.
using DistanceMatrix = vector<vector<int>>;
// Coordinate of symbols. Given that "Coordinate example", "example.first"
// represents the row index and "example.second" represents the column index.
using Coordinate = pair<int, int>;

using StrictBitset = bitset<32>;
using GroupIndexValueMapping = unordered_map<size_t, int>;
using GroupValue = unordered_map<StrictBitset, GroupIndexValueMapping>;


const char kStartSymbol = 'S';
const char kGoalSymbol = 'G';
const char kCheckpointSymbol = '@';
const char kCloseBlockSymbol = '#';


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


class DistanceMatrixGenerator {
 public:
  bool Generate(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets);

  DistanceMatrix distance_matrix_;

 private:
  bool FindShortestPathFromSingleSource(
      const vector<string> &orienteering_map,
      const vector<Coordinate> &targets,
      const size_t &source_index);

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


class TSPCalculator {
 public:
  int CalculateMinLength(const DistanceMatrix &distance_matrix);

 private:
  vector<StrictBitset> GenerateSetsOfCheckpoints(
      const int &length, const int &owned_elements);

  void UpdateMinLengths(
      const StrictBitset &checkpoint_set,
      const DistanceMatrix &distance_matrix,
      GroupValue *min_lengths_ptr);
};


// ============================================================================
// Definition of classes.
// ============================================================================
namespace std {

template <>
struct hash<Coordinate> {
  using result_type = size_t;
  using argument_type = Coordinate;
  // call operator.
  size_t operator()(const Coordinate &target) const {
    // XOR of hash results.
    return hash<int>()(target.first) ^ hash<int>()(target.second);
  }
};

}  // namespace std


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
  size_t width = 0, height = 0;
  *in_ptr >> width >> height;

  for (size_t counter = 0; counter != height; ++counter) {
    string line;
    *in_ptr >> line;
    orienteering_map_.push_back(line);
  }

  // find start, goal and checkpoints.
  for (size_t row_index = 0; row_index != height; ++row_index) {
    for (size_t column_index = 0; column_index != width; ++column_index) {
      RecordCoordinate(row_index, column_index);
    }
  }
}

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

inline bool DistanceMatrixGenerator::IsValid(
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
  for (const auto &next_coordinate : {Coordinate(x - 1, y),  // up.
                                      Coordinate(x + 1, y),  // down.
                                      Coordinate(x, y - 1),  // left.
                                      Coordinate(x, y + 1)}) {  // right.
    if (IsValid(next_coordinate, row_size, column_size)) {
      next_coordinates.push_back(
          std::move(next_coordinate));
    }
  }
  return next_coordinates;
}

// Find the shortest paths from single source to all the others by using BFS.
bool DistanceMatrixGenerator::FindShortestPathFromSingleSource(
    const vector<string> &orienteering_map,
    const vector<Coordinate> &targets,
    const size_t &source_index) {
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
  size_t searched_targets = 0;

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
      const size_t target_index = distance(
          targets.cbegin(),
          find(targets.cbegin(), targets.cend(), coordinate));
      // now, we can fill element of the distance_matrix_, pointed by
      // (source_index, target_index), with the value distance.
      distance_matrix_[source_index][target_index] = current_distance;
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

bool DistanceMatrixGenerator::Generate(
    const vector<string> &orienteering_map,
    const vector<Coordinate> &targets) {
  // function to be called concurrently.
  using MemberFunction = bool (
      DistanceMatrixGenerator *,
      const vector<string> &,
      const vector<Coordinate> &,
      const size_t &);
  function<MemberFunction> fcn =
      &DistanceMatrixGenerator::FindShortestPathFromSingleSource;
  // init distance_matrix_.
  const int target_size = targets.size();
  distance_matrix_ = InitMatrix(target_size, target_size, 0);

  // fill the matrix.
  vector<future<bool>> future_objs;
  for (size_t source_index = 0; source_index != targets.size(); ++source_index) {
    // multithreading.
    future_objs.push_back(
        async(fcn, this, orienteering_map, targets, source_index));
  }
  // make sure all tasks finished.
  for (auto &future_obj : future_objs) {
    if (!future_obj.get()) { return false; }
  }
  // all is well.
  return true;
}

// length would always > 0.
vector<StrictBitset> TSPCalculator::GenerateSetsOfCheckpoints(
    const int &length, const int &owned_elements) {
  vector<StrictBitset> checkpoint_sets;
  // seed of permutation.
  vector<bool> seed(length, false);
  fill(seed.begin(), seed.begin() + owned_elements, true);
  // generate permutations.
  do {
    StrictBitset current_set;
    for (size_t index = 0; index != seed.size(); ++index) {
      if (seed[index]) { current_set.set(index); }
    }
    checkpoint_sets.push_back(std::move(current_set));
  } while (next_permutation(seed.rbegin(), seed.rend()));
  return checkpoint_sets;
}

void TSPCalculator::UpdateMinLengths(
    const StrictBitset &checkpoint_set,
    const DistanceMatrix &distance_matrix,
    GroupValue *min_lengths_ptr) {

  vector<size_t> indices;
  for (size_t index = 0; index != checkpoint_set.size(); ++index) {
    if (checkpoint_set[index]) {
      indices.push_back(index);
    }
  }

  for (const size_t &index : indices) {
    // make previous subset.
    StrictBitset previous_subset(checkpoint_set);
    previous_subset.reset(index);

    // init total_minimum to max value.
    int total_minimum = numeric_limits<int>::max();
    // loop over the rest of indices.
    for (const size_t &other_index : indices) {
      if (other_index == index) { continue; }
      // access previous result.
      const int &previous_length =
          (*min_lengths_ptr)[previous_subset][other_index];
      // calculate current length.
      int current_length = previous_length
                           + distance_matrix[index][other_index];
      if (current_length < total_minimum) {
        total_minimum = current_length;
      }
    }
    // update min_lengths.
    (*min_lengths_ptr)[checkpoint_set][index] = total_minimum;
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

  // init.
  GroupValue min_lengths;
  auto init_sets = GenerateSetsOfCheckpoints(checkpoint_size, 1);
  for (size_t checkpoint_index = 0;
       checkpoint_index != checkpoint_size; ++checkpoint_index) {

    const auto &init_set = init_sets[checkpoint_index];
    min_lengths[init_set][checkpoint_index] =
        distance_matrix[source_index][checkpoint_index];
  }
  // internal step.
  for (size_t subset_size = 2;
       subset_size <= checkpoint_size; ++subset_size) {
    
    for (const auto &checkpoint_set 
         : GenerateSetsOfCheckpoints(checkpoint_size, subset_size)) {
      UpdateMinLengths(checkpoint_set, distance_matrix, &min_lengths);
    }
  }
  // final step.
  const StrictBitset all_checkpoints =
      GenerateSetsOfCheckpoints(checkpoint_size, checkpoint_size).front();

  int total_minimum = numeric_limits<int>::max();
  for (size_t checkpoint_index = 0;
       checkpoint_index != checkpoint_size; ++checkpoint_index) {
    int current_length = min_lengths[all_checkpoints][checkpoint_index]
                         + distance_matrix[checkpoint_index][goal_index];
    if (current_length < total_minimum) {
      total_minimum = current_length;
    }
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

  DistanceMatrixGenerator generator;
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
