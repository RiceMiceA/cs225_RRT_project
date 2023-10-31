/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include "planner.h"

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



