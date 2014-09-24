// ============================================================================
//
//       Filename:  test_tsp.cc
//
//    Description:  
//
//        Version:  1.0
//        Created:  09/25/2014 00:42:11
//       Revision:  none
//       Compiler:  g++
//
//         Author:  Zhan Haoxun (huntzhan), programmer.zhx@gmail.com
//   Organization:  
//
// ============================================================================
#include "gtest/gtest.h"
#include "orienteering.cpp"

TEST(tsp, example_1) {
  DistanceMatrix dm = {
    {0, 4},
    {4, 0}};
  TSPCalculator calculator;
  EXPECT_EQ(calculator.CalculateMinLength(dm), 4);
}

TEST(tsp, example_2) {
  DistanceMatrix dm = {
    {0, 1, 2, 5},
    {1, 0, 3, 6},
    {2, 3, 0, 3},
    {5, 6, 3, 0}};
  TSPCalculator calculator;
  EXPECT_EQ(calculator.CalculateMinLength(dm), 9);
}

TEST(tsp, example_3) {
  DistanceMatrix dm = {
    {0, 4, 2},
    {4, 0, 2},
    {2, 2, 0}};
  TSPCalculator calculator;
  EXPECT_EQ(calculator.CalculateMinLength(dm), 6);
}
