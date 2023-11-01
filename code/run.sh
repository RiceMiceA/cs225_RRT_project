# Compile the C++ code
clang++ -std=c++20 planner.cpp -o planner.out
echo compiled successful
# Run the C++ code with the given arguments
./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 2 myOutput.txt
echo test finished
# Run the Python script to generate the GIF
python3 visualizer.py myOutput.txt --gifFilepath=myGif.gif
echo gif done