# g++-4.8 -std=c++11 -O2 -o a.out orienteering.cpp

echo "Run data1.in, Answer is 6"
time ./a.out < data1.in
echo "Run data2.in, Answer is 9"
time ./a.out < data2.in
echo "Run data3.in, Answer is 4"
time ./a.out < data3.in
echo "Run data4.in, Answer is 484"
time ./a.out < data4.in
echo "Run data5.in, No Answer"
time ./a.out < data5.in || echo "return -1"
echo "Run data6.in, No Answer"
time ./a.out < data6.in || echo "return -1"
echo "Run data7.in, Answer is 14"
time ./a.out < data7.in
echo "Run data8.in, Answer is 20"
time ./a.out < data8.in
echo "Run data9.in, Answer is 1"
time ./a.out < data9.in
echo "Run data10.in, Answer is 2"
time ./a.out < data10.in
echo "Run data11.in, Answer is 1"
time ./a.out < data11.in
