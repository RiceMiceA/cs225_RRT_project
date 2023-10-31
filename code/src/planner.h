#pragma once
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

tuple<double*, int, int> loadMap(string filepath);
vector<string> split(const string& str, const string& delim);
double* doubleArrayFromString(string str);
bool equalDoubleArrays(double* v1, double *v2, int size);

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size);
void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);
void get_current_point(bresenham_param_t *params, int *x, int *y);
int get_next_point(bresenham_param_t *params);
int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size);
int IsValidArmConfiguration(
	double* angles, int numofDOFs, double*	map,
	int x_size, int y_size);

class Tree {
    public:
        Tree() = default;
        Tree(double* q_init): graph_(), root_(q_init) {
            if (root_ == NULL) {
                root_ = q_init;
            }
            graph_[q_init] = list<double*>(); 
        }
        ~Tree() = default;
        void AddVertex(double* q) { graph_[q] = list<double*>(); }
        void AddEdge(double* q1, double* q2) {		// q1 -- parent, q2 -- child
            graph_[q1].push_back(q2); 
            graph_[q2].push_back(q1); 
            parents_[q2] = q1;
        }
        unordered_map<double*,list<double*>>& GetGraph() { return graph_; }
        double* GetRoot() const { return root_; }
        unordered_map<double*,double*>& GetParents() { return parents_; }

    private:
        unordered_map<double*,list<double*>> graph_;
        double* root_ = NULL;
        unordered_map<double*,double*> parents_;
};

class Graph {
    public:
        Graph() = default;
        Graph(double* q_init): graph_() { graph_[q_init] = list<double*>(); }
        ~Graph() = default;
        void AddVertex(double* q) { graph_[q] = list<double*>(); }
        void AddEdge(double* q1, double* q2) {
            graph_[q1].push_back(q2); 
            graph_[q2].push_back(q1); 
        }
        unordered_map<double*,list<double*>>& GetGraph() { return graph_; }
        list<double*>& GetAdj(double* vertex) { return graph_[vertex]; }

    private:
        unordered_map<double*,list<double*>> graph_;
};

double fRand();
double* RandomConfig(int numofDOFs);

enum class Status {
	Reached = 0,
	Advanced = 1,
	Trapped = 2
};

double EuclidianDist(double* q1, double* q2, int numofDOFs) ;
pair<double*,double> NearestNeighbor(Tree& T, double* q, int numofDOFs) ;
bool NewConfig(double* q, double* q_near, double* map, int x_size, int y_size, double dist, double epsilon, int numofDOFs, double*& q_new);
Status Extend(Tree& T, double* q, int numofDOFs, double epsilon, double* map, int x_size, int y_size, double*& q_new);

// Libraries of RRT
static void RRT(
	double* map,
	int x_size, int y_size,
	double* armstart_anglesV_rad,	// init
	double* armgoal_anglesV_rad,	// goal
	int numofDOFs,
	double*** plan,
	int* planlength,
	int K = 1000,
	double epsilon = 0.3
);
