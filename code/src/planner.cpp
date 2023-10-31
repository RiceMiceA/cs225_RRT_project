/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
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

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}

double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            return false;
        }
    }
    return true;
}

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

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}


int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(
	double* angles, int numofDOFs, double*	map,
	int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

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

double fRand() {
	double fMin = 0, fMax = 6;
    double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

double* RandomConfig(int numofDOFs) {
	double* config = new double[numofDOFs];
	for (int i = 0; i < numofDOFs; ++i) {
		config[i] = fRand();
	}
	return config;
}

enum class Status {
	Reached = 0,
	Advanced = 1,
	Trapped = 2
};

double EuclidianDist(double* q1, double* q2, int numofDOFs) {
	double res = 0;
	for (int i = 0; i < numofDOFs; ++i) {
		res += (q1[i] - q2[i])*(q1[i] - q2[i]);
	}
	return sqrt(res);
}

pair<double*,double> NearestNeighbor(Tree& T, double* q, int numofDOFs) {
	double* nearest = T.GetRoot();
	double min_dist = EuclidianDist(nearest, q, numofDOFs);
	for (auto & [key, val] : T.GetGraph()) {
		double dist = EuclidianDist(key, q, numofDOFs);
		if (dist < min_dist) {
			min_dist = dist;
			nearest = key;
		}
	}
	return {nearest, min_dist};
}

bool NewConfig(double* q, double* q_near, double* map, int x_size, int y_size, double dist, double epsilon, int numofDOFs, double*& q_new) {
	if (dist < epsilon) {
		for (int i = 0; i < numofDOFs; ++i) {
			q_new[i] = q[i];
		}
		return IsValidArmConfiguration(q_new, numofDOFs, map, x_size, y_size);
	}
	double* dir = new double[numofDOFs];
	for (int i = 0; i < numofDOFs; ++i) {
		dir[i] = 0;
	}
	double norm = EuclidianDist(q, q_near, numofDOFs);
	for (int i = 0; i < numofDOFs; ++i) {
		dir[i] = epsilon * (q[i] - q_near[i]) / norm;
	}
	for (int i = 0; i < numofDOFs; ++i) {
		q_new[i] = q_near[i] + dir[i];
	}
	return IsValidArmConfiguration(q_new, numofDOFs, map, x_size, y_size);
}

Status Extend(Tree& T, double* q, int numofDOFs, double epsilon, double* map, int x_size, int y_size, double*& q_new) {
	pair<double*,double> p = NearestNeighbor(T, q, numofDOFs);
	double* q_near = p.first; double dist = p.second;
	bool new_config = NewConfig(q, q_near, map, x_size, y_size, dist, epsilon, numofDOFs, q_new);
	if (new_config) {
		double* new_vertex = new double[numofDOFs];
		for (int i = 0; i < numofDOFs; ++i) {
			new_vertex[i] = q_new[i];
		}
		T.AddVertex(new_vertex);
		T.AddEdge(q_near, new_vertex);
		if (equalDoubleArrays(q_new, q, numofDOFs)) {
			return Status::Reached;
		} else {
			return Status::Advanced;
		}
	}
	return Status::Trapped;
}

Status Connect(Tree& T, double* q, int numofDOFs, double epsilon, double* map, int x_size, int y_size, double*& q_new) {
	Status S;
	do {
		S = Extend(T, q, numofDOFs, epsilon, map, x_size, y_size, q_new);
	} while (S == Status::Advanced);
	return S;
}

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
) {
	//no plan by default
	*plan = NULL;
	*planlength = 0;

	// build graph
	Tree T(armstart_anglesV_rad);
	for (int k = 1; k <= K; ++k) {
		double* q_rand = RandomConfig(numofDOFs);
		double* q_new = new double[numofDOFs];
		for (int i = 0; i < numofDOFs; ++i) {
			q_new[i] = 0;
		}
		Extend(T, q_rand, numofDOFs, epsilon, map, x_size, y_size, q_new);
	}

	// generate plan
	vector<double*> plan_rev;
	plan_rev.push_back(armgoal_anglesV_rad);
	pair<double*,double> p = NearestNeighbor(T, armgoal_anglesV_rad, numofDOFs);
	double* curr = p.first; double dist = p.second;
	unordered_map<double*,double*>& parents = T.GetParents();
	while (parents.find(curr) != parents.end()) {	// while not reach root_(init_config)
		curr = parents[curr];
		plan_rev.push_back(curr);
	}
	plan_rev.push_back(armstart_anglesV_rad);
	*planlength = plan_rev.size();
	*plan = new double*[*planlength];
	for (int i = 0; i < *planlength; ++i) {
		(*plan)[i] = new double[numofDOFs];
		for (int j = 0; j < numofDOFs; ++j) {
			(*plan)[i][j] = plan_rev[*planlength - i -1][j];
		}
	}
	return;
}

static void RRTConnect(
	double* map,
	int x_size, int y_size,
	double* armstart_anglesV_rad,	// init
	double* armgoal_anglesV_rad,	// goal
	int numofDOFs,
	double*** plan,
	int* planlength,
	int K = 10000,
	double epsilon = 0.2
) {
	//no plan by default
	*plan = NULL;
	*planlength = 0;

	Tree Ta(armstart_anglesV_rad);
	Tree Tb(armgoal_anglesV_rad);

	for (int k = 1; k <= K; ++k) {
		double* q_rand = RandomConfig(numofDOFs);
		double* q_new = new double[numofDOFs];
		double* q_reach = new double[numofDOFs];
		for (int i = 0; i < numofDOFs; ++i) {
			q_new[i] = 0;
			q_reach[i] = 0;
		}
		if (!(Extend(Ta, q_rand, numofDOFs, epsilon, map, x_size, y_size, q_new) == Status::Trapped)) {
			if (Connect(Tb, q_new, numofDOFs, epsilon, map, x_size, y_size, q_reach) == Status::Reached) {
				double* reach = NULL;
				for (auto & [key, val] : Tb.GetGraph()) {
					if (equalDoubleArrays(q_new, key, numofDOFs)) {
						reach = key;
						break;
					}
				}
				auto Ta_parents = Ta.GetParents();
				auto Tb_parents = Tb.GetParents();
				vector<double*> plan_tmp;
				double* curr = reach;
				plan_tmp.push_back(curr);
				if (equalDoubleArrays(Ta.GetRoot(), armgoal_anglesV_rad, numofDOFs)) { 	// Ta rooted at goal
					while (Tb_parents.find(curr) != Tb_parents.end()) {
						curr = Tb_parents[curr];
						plan_tmp.push_back(curr);
					}
					plan_tmp.push_back(armstart_anglesV_rad);
					std::reverse(plan_tmp.begin(), plan_tmp.end());
					curr = reach;
					while (Ta_parents.find(curr) != Ta_parents.end()) {
						curr = Ta_parents[curr];
						plan_tmp.push_back(curr);
					}
					plan_tmp.push_back(armgoal_anglesV_rad);
				} else if (equalDoubleArrays(Tb.GetRoot(), armgoal_anglesV_rad, numofDOFs)) {	// Tb rooted at goal
					while (Ta_parents.find(curr) != Ta_parents.end()) {
						curr = Ta_parents[curr];
						plan_tmp.push_back(curr);
					}
					plan_tmp.push_back(armstart_anglesV_rad);
					std::reverse(plan_tmp.begin(), plan_tmp.end());
					curr = reach;
					while (Tb_parents.find(curr) != Tb_parents.end()) {
						curr = Tb_parents[curr];
						plan_tmp.push_back(curr);
					}
					plan_tmp.push_back(armgoal_anglesV_rad);
				}
				*planlength = plan_tmp.size();
				*plan = new double*[*planlength];
				for (int i = 0; i < *planlength; ++i) {
					(*plan)[i] = new double[numofDOFs];
					for (int j = 0; j < numofDOFs; ++j) {
						(*plan)[i][j] = plan_tmp[i][j];
					}
				}
				cout << "RRT-Connect found a path" << endl;
				return;
			}
		}
		std::swap(Ta, Tb);
	}
	cout << "RRT-Connect Failed" << endl;
	return;
}

static void RRTStar(
	double* map,
	int x_size, int y_size,
	double* armstart_anglesV_rad,	// init
	double* armgoal_anglesV_rad,	// goal
	int numofDOFs,
	double*** plan,
	int* planlength,
	int K = 10000,
	double epsilon = 0.2
) {
	return;
}

vector<double*> Neighborhood(Graph& G, double* q, int numofDOFs, double radius=0.75) {
	vector<double*> neighbors;
	for (auto & [key, val] : G.GetGraph()) {
		if (EuclidianDist(key, q, numofDOFs) <= radius) {
			neighbors.push_back(key);
		}
	}
	return neighbors;
}

bool CanConnect(double* q1, double* q2) {
	// TODO()
	return true;
}

double Heuristic(double* q1, double* q2, int numofDOFs) {
	return EuclidianDist(q1, q2, numofDOFs);
}

vector<double*> AStar(Graph& G, double* init, double* goal, int numofDOFs) {
	unordered_set<double*> closed;
	priority_queue<pair<double,double*>,
				   vector<pair<double,double*>>,
				   std::greater<pair<double,double*>>> open;	// f, config
	unordered_map<double*,double> g;	// config --> g
	unordered_map<double*,double*> prev;	// config --> prev config

	open.push({0, init}); g[init] = 0;
	while (closed.find(goal) == closed.end() && !open.empty()) {
		auto [f, q] = open.top(); open.pop(); closed.insert(q);
		for (auto & adj : G.GetAdj(q)) {
			double edge_cost = EuclidianDist(q, adj, numofDOFs);
			if (closed.find(adj) == closed.end() &&		// not closed
			   (g.find(adj) == g.end() || g[adj] > edge_cost + g[q])) { 
			    g[adj] = edge_cost + g[q];
				prev[adj] = q;
				open.push({g[adj] + Heuristic(adj, goal, numofDOFs), adj});
			}
		}
	}
	vector<double*> plan;
	if (closed.find(goal) == closed.end()) {
		cout << "A* failed to find a path" << endl;
		return plan;
	}
	double* curr = goal;
	do {
		plan.push_back(curr);
		curr = prev[curr];
	} while (curr != init);
	plan.push_back(init);
	return plan;
}

static void PRM(
	double* map,
	int x_size, int y_size,
	double* armstart_anglesV_rad,	// init
	double* armgoal_anglesV_rad,	// goal
	int numofDOFs,
	double*** plan,
	int* planlength,
	int N = 10000
) {
	Graph G; int i = 0;
	vector<double*> neighbors;
	// random sampling
	while (i < N) {
		double* q = RandomConfig(numofDOFs);
		if (IsValidArmConfiguration(q, numofDOFs, map, x_size, y_size)) {
			G.AddVertex(q); ++i;
			neighbors = Neighborhood(G, q, numofDOFs);
			for (auto & nbr : neighbors) {
				if (CanConnect(nbr, q)) {
					G.AddEdge(nbr, q);
				}
			}
		}
	}
	// add init
	G.AddVertex(armstart_anglesV_rad);
	neighbors = Neighborhood(G, armstart_anglesV_rad, numofDOFs);
	for (auto & nbr : neighbors) {
		if (CanConnect(nbr, armstart_anglesV_rad)) {
			G.AddEdge(nbr, armstart_anglesV_rad);
		}
	}
	// add goal
	G.AddVertex(armgoal_anglesV_rad);
	neighbors = Neighborhood(G, armgoal_anglesV_rad, numofDOFs);
	for (auto & nbr : neighbors) {
		if (CanConnect(nbr, armgoal_anglesV_rad)) {
			G.AddEdge(nbr, armgoal_anglesV_rad);
		}
	}
	
	// run A*
	vector<double*> plan_rev = AStar(G, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs);
	// retrieve plan
	*planlength = plan_rev.size();
	*plan = new double*[*planlength];
	for (int i = 0; i < *planlength; ++i) {
		(*plan)[i] = new double[numofDOFs];
		for (int j = 0; j < numofDOFs; ++j) {
			(*plan)[i][j] = plan_rev[*planlength - i - 1][j];
		}
	}
	return;
}

static void planner(
	double* map,
	int x_size,
	int y_size,
	double* armstart_anglesV_rad,
	double* armgoal_anglesV_rad,
	int numofDOFs,
	double*** plan,
	int* planlength) {

	//no plan by default
	*plan = NULL;
	*planlength = 0;

    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("The arm is already at the goal\n");
        return;
    }
	int countNumInvalid = 0;
	*plan = new double*[numofsamples];
    for (i = 0; i < numofsamples; i++){
		(*plan)[i] = new double[numofDOFs];
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
			++countNumInvalid;
        }
    }
	printf("Linear interpolation collided at %d instances across the path\n", countNumInvalid);
    *planlength = numofsamples;
    return;
}

enum class Planner {
	RRT = 0,
	RRTCONNECT = 1,
	RRTSTAR = 2,
	PRM = 3
};

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
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

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;
	// RRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// if (whichPlanner == (int)Planner::RRT) {
	RRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);		
	// } else if (whichPlanner == (int)Planner::RRTCONNECT) {
	// 	RRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// } else if (whichPlanner == (int)Planner::RRTSTAR) {
	// 	RRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// } else if (whichPlanner == (int)Planner::PRM) {
	// 	PRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// } else {
	// 	planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as the 
	//// grading script will not work.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
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
