// ============================================================================
//
//       Filename:  test_search.cc
//
//    Description:  
//
//        Version:  1.0
//        Created:  09/24/2014 20:13:25
//       Revision:  none
//       Compiler:  g++
//
//         Author:  Zhan Haoxun (huntzhan), programmer.zhx@gmail.com
//   Organization:  
//
// ============================================================================
#include "gtest/gtest.h"
#include "orienteering.cpp"

#include <vector>
#include <utility>
#include <string>

using std::vector;
using std::make_pair;
using std::string;


TEST(shortest_path, simple) {
  vector<string> orienteering_map = {
      "#####",
      "#.S.#",
      "#..G#",
      "#@..#",
      "#####"};
  vector<Coordinate> targets = {
    make_pair(1, 2),  // S.
    make_pair(2, 3),  // G.
    make_pair(3, 1)}; // @.

  DistanceMatrixGenerator generator;
  bool run_flag = generator.Generate(orienteering_map, targets);
  EXPECT_TRUE(run_flag);

  const auto &dm = generator.distance_matrix;
  EXPECT_EQ(dm[0][1], 2);
  EXPECT_EQ(dm[1][0], 2);

  EXPECT_EQ(dm[0][2], 3);
  EXPECT_EQ(dm[2][0], 3);

  EXPECT_EQ(dm[1][2], 3);
  EXPECT_EQ(dm[2][1], 3);
}


TEST(shortest_path, error_case) {
  vector<string> orienteering_map = {
      "#####",
      "#.S.#",
      "###G#",
      "#@#.#",
      "#####"};
  vector<Coordinate> targets = {
    make_pair(1, 2),  // S.
    make_pair(2, 3),  // G.
    make_pair(3, 1)}; // @.

  DistanceMatrixGenerator generator;
  bool run_flag = generator.Generate(orienteering_map, targets);
  EXPECT_FALSE(run_flag);
}
