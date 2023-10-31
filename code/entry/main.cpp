/**
 * @file main.cpp
 * A fake 'run-script' to get time estimates for implemented algorithms
 * For other approaches to time your code (for benchmarking) see:
 * https://levelup.gitconnected.com/8-ways-to-measure-execution-time-in-c-c-48634458d0f9
 */

#include <algorithm>
#include <vector>
#include <iostream>
#include <utility>

#include "planner.h"
/**
 * Main routine.
 * Reads in a specified dataset and outputs the time to do 'algorithm' on it
 */
int main(int argc, char** argv)
{
   srand(time(NULL));

	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
	   !IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	double** plan = NULL;
	int planlength = 0;
	RRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);		

    // Solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
