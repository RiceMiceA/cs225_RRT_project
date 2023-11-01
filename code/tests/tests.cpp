#include <catch2/catch_test_macros.hpp>

#include "planner.cpp"

#include <sstream>
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

#include <unordered_map>
#include <list>
#include <random>
#include <time.h>
#include <queue>
#include <unordered_set>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
// #define RRT         0
// #define RRTCONNECT  1
// #define RRTSTAR     2
// #define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;
using std::unordered_map;
using std::list;
using std::pair;
using std::unordered_set;
using std::priority_queue;


/**
 * Reads in the dataset smallset and searches for a known matching pattern CACATCTA
 * The pattern exists (1) time in the csv on row 1:
 * TGATTTTAAAAAAACACTTAACACATCTAGATAGAATAGTACTCTGCCCTATTTGAGGGAACAGTCTCAAACNATGAAGTACATGATATTTAATGCCCTA
 */
TEST_CASE("Simple Initialization Parameter Checking", "[size = 50 x 50]")
{
    // define input argv
    const char* test = "../../data/map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt";
    const char** argv = &test;
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

    REQUIRE(IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size) && IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size));

}

TEST_CASE("Advanced Initialization Parameter Checking", "[size = 50 x 50]")
{
    // define input argv
    const char* test = "../../data/map2.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt";
    const char** argv = &test;
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

    REQUIRE(IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size) && IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size));

}

TEST_CASE("Extension Checking", "[size = 50 x 50]")
{
    // define input argv
    const char* test = "../../data/map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt";
    const char** argv = &test;
    srand(time(NULL));
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

    REQUIRE(IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size) && IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size));

}

TEST_CASE("Final Result Testing", "[size = 50 x 50]")
{
	// define input argv
    const char* test = "../../data/map2.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt";
    const char** argv = &test;
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

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}
    REQUIRE(equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs) && equalDoubleArrays(plan[0], startPos, numOfDOFs));

}

/**
 * Real tests will include things you havent implemented yet
 * For example, I know that I have not yet written blackbox_count yet
 * but I have a good idea of what the input / output of this function should be...
 * 
 * Reads in the dataset smallset and searches for a known matching pattern CCTT
 * The pattern exists (51) time in the csv. Accordingly it should pass a test to find it 51 times
 * and fail to find it 52 times.
 */
TEST_CASE("Larger Final Result Testing", "[size = 100 x 100]")
{
    const char* test = "../../data/map3.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt";
    const char** argv = &test;
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

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}
    REQUIRE(equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs) && equalDoubleArrays(plan[0], startPos, numOfDOFs));
}
