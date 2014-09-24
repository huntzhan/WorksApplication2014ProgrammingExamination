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

#include <iostream>
#include <vector>


using std::cin;
using std::cout;
using std::endl;
using std::vector;

// Two-dimension matrix.
using DistanceMatrix = vector<vector<int>>;


// Strategy:
// 1. Find the shortest path between each pairs of targets(start, goal,
// checkpoint), based on BFS.
// 2. The problem were transformed to be a TSP problem. Solve the problem with 
// dynamic programming.





// Skeleton code for the examination.
class Orienteering {
 public:
  void main();
};

void Orienteering::main() {
}

int main(int argc, char* argv[]) {
  Orienteering o;
  o.main();
  return 0;
}
