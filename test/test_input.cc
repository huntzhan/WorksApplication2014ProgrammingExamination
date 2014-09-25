// ============================================================================
//
//       Filename:  test_input.cc
//
//    Description:  
//
//        Version:  1.0
//        Created:  09/24/2014 13:54:29
//       Revision:  none
//       Compiler:  g++
//
//         Author:  Zhan Haoxun (huntzhan), programmer.zhx@gmail.com
//   Organization:  
//
// ============================================================================
#include "gtest/gtest.h"
#include "orienteering.cpp"

#include <sstream>
#include <utility>


using std::endl;
using std::istringstream;
using std::ostringstream;
using std::make_pair;


TEST(project, simple_intput) {
  ostringstream ostrm;
  ostrm << 5 << " " <<  5 << endl;
  ostrm << "#####" << endl;
  ostrm << "#.S.#" << endl;
  ostrm << "#..G#" << endl;
  ostrm << "#@@@#" << endl;
  ostrm << "#####" << endl;
  istringstream istrm(ostrm.str());

  InputHandler handler;
  handler.ReadFromInputStream(&istrm);
  EXPECT_EQ(handler.start_, make_pair(1, 2));
  EXPECT_EQ(handler.goal_, make_pair(2, 3));
  EXPECT_EQ(handler.checkpoints_.size(), 3);
}
