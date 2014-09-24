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
#include <vector>
#include <utility>


using std::istream;
using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::pair;
using std::make_pair;
using std::string;

// Two-dimension matrix.
using DistanceMatrix = vector<vector<int>>;
// Coordinate of symbols. Given that "Coordinate example", "example.first"
// represents the row index and "example.second" represents the column index.
using Coordinate = pair<int, int>;


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
