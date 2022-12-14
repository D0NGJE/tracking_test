#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <set>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/types.hpp>
#include <set>

#include "KalmanTracker.h"
#include "Hungarian.h"

typedef struct Point_{
    pcl::PointXYZI p;
    int clusterID;
}Point;

typedef struct TrackingBox
{
	int frame;
	int id;
    cv::Rect_<float> box;
}TrackingBox;

bool point_compare(pcl::PointXYZI a, pcl::PointXYZI b){
    return a.z < b.z;
}



pcl::PointCloud<pcl::PointXYZI>::Ptr g_seeds_pc(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr g_ground_pc(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr g_not_ground_pc(new pcl::PointCloud<pcl::PointXYZI>);

///////////////////////////////////////////////////////////////////////////////
// KalmanTracker.cpp: KalmanTracker Class Implementation Declaration

int KalmanTracker::kf_count = 0;

// initialize Kalman filter
void KalmanTracker::init_kf(StateType stateMat)
{
	int stateNum = 7;
	int measureNum = 4;
	kf = cv::KalmanFilter(stateNum, measureNum, 0);

	measurement = cv::Mat::zeros(measureNum, 1, CV_32F); // 4 * 1
    // std::cout << measurement << std::endl;

    cv::Mat_<float> A;
    cv::Mat_<float> H;
    cv::Mat_<float> Q;
    cv::Mat_<float> R;

    cv::Mat_<float> x;
    cv::Mat_<float> P;
	// kf.transitionMatrix = cv::Mat_<float>(stateNum, stateNum) <<
	// 	1, 0, 0, 0, 1, 0, 0,
	// 	0, 1, 0, 0, 0, 1, 0,
	// 	0, 0, 1, 0, 0, 0, 1,
	// 	0, 0, 0, 1, 0, 0, 0,
	// 	0, 0, 0, 0, 1, 0, 0,
	// 	0, 0, 0, 0, 0, 1, 0,
	// 	0, 0, 0, 0, 0, 0, 1;
    A = cv::Mat::eye(7,7, CV_32F);
    A[0][4] = 1;
    A[1][5] = 1;
    A[2][6] = 1;
    kf.transitionMatrix = A;

    H = cv::Mat::eye(4,7, CV_32F);
    kf.measurementMatrix = H;

    Q = 3 * cv::Mat::eye(7,7, CV_32F);
    kf.processNoiseCov = Q;

    R = 7 * cv::Mat::eye(4,4, CV_32F);
    kf.measurementNoiseCov = R;

    x = cv::Mat::zeros(4, 1, CV_32F);
    measurement = x;

    P = 20 * cv::Mat::eye(7,7, CV_32F);
    kf.errorCovPost = P;

	// setIdentity(kf.measurementMatrix); // 4 * 7
    // std::cout << kf.measurementMatrix << std::endl;
	// setIdentity(kf.processNoiseCov, cv::Scalar::all(3)); // 7 * 7
    // std::cout << kf.processNoiseCov << std::endl;
	// setIdentity(kf.measurementNoiseCov, cv::Scalar::all(7)); // 4 * 4
    // std::cout << kf.measurementNoiseCov << std::endl;
	// setIdentity(kf.errorCovPost, cv::Scalar::all(20)); // 7 * 7
    // std::cout << kf.errorCovPost << std::endl;
	
	// initialize state vector with bounding box in [cx,cy,s,r] style
	kf.statePost.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
	kf.statePost.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
	kf.statePost.at<float>(2, 0) = stateMat.area();
	kf.statePost.at<float>(3, 0) = stateMat.width / stateMat.height;
}

// Predict the estimated bounding box.
StateType KalmanTracker::predict()
{
	// predict
	cv::Mat p = kf.predict();
    // std::cout << "predict : " << p << std::endl;
	m_age += 1;

	if (m_time_since_update > 0)
		m_hit_streak = 0;
	m_time_since_update += 1;

	StateType predictBox = get_rect_xysr(p.at<float>(0, 0), p.at<float>(1, 0), p.at<float>(2, 0), p.at<float>(3, 0));

	m_history.push_back(predictBox);
	return m_history.back();
}

StateType KalmanTracker::predict2()
{
    cv::Mat p = kf.predict();
	// predict
    cv::Mat_<float> A;
    cv::Mat_<float> H;
    cv::Mat_<float> Q;
    cv::Mat_<float> R;

    cv::Mat_<float> x;
    cv::Mat_<float> P;

    bool firstRun = false;

    if(firstRun = false){
        A = cv::Mat::eye(7,7, CV_32F);
        A[0][4] = 1;
        A[1][5] = 1;
        A[2][6] = 1;

        H = cv::Mat::eye(4,7, CV_32F);

        Q = 3 * cv::Mat::eye(7,7, CV_32F);

        R = 7 * cv::Mat::eye(4,4, CV_32F);
        
        x = cv::Mat::zeros(7, 1, CV_32F);

        P = 20 * cv::Mat::eye(7,7, CV_32F);

        firstRun = true;
    }

    // cv::Mat_<float> xp = A * x;
    std::cout << A << std::endl;
    


	m_age += 1;

	if (m_time_since_update > 0)
		m_hit_streak = 0;
	m_time_since_update += 1;

	StateType predictBox = get_rect_xysr(p.at<float>(0, 0), p.at<float>(1, 0), p.at<float>(2, 0), p.at<float>(3, 0));

	m_history.push_back(predictBox);
	return m_history.back();
}

// Update the state vector with observed bounding box.
void KalmanTracker::update(StateType stateMat)
{
	m_time_since_update = 0;
	m_history.clear();
	m_hits += 1;
	m_hit_streak += 1;

	// measurement
	measurement.at<float>(0, 0) = stateMat.x + stateMat.width / 2;
	measurement.at<float>(1, 0) = stateMat.y + stateMat.height / 2;
	measurement.at<float>(2, 0) = stateMat.area();
	measurement.at<float>(3, 0) = stateMat.width / stateMat.height;

	// update
	kf.correct(measurement);
}

// Return the current state vector
StateType KalmanTracker::get_state()
{
	cv::Mat s = kf.statePost;
	return get_rect_xysr(s.at<float>(0, 0), s.at<float>(1, 0), s.at<float>(2, 0), s.at<float>(3, 0));
}

// Convert bounding box from [cx,cy,s,r] to [x,y,w,h] style.
StateType KalmanTracker::get_rect_xysr(float cx, float cy, float s, float r) // cx, cy = ?????????, s=??????=w*h  r=?????????=w/h
{
	float w = sqrt(s * r);
	float h = s / w;
	float x = (cx - w / 2);
	float y = (cy - h / 2);

	if (x < 0 && cx > 0)
		x = 0;
	if (y < 0 && cy > 0)
		y = 0;

	return StateType(x, y, w, h); // x, y = ?????? ?????? ??????, w = ??????, h = ??????
}

// Computes IOU between two bounding boxes
double GetIOU(cv::Rect_<float> bb_test, cv::Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Hungarian.cpp: Implementation file for Class HungarianAlgorithm.
HungarianAlgorithm::HungarianAlgorithm(){}
HungarianAlgorithm::~HungarianAlgorithm(){}
//********************************************************//
// A single function wrapper for solving assignment problem.
//********************************************************//
double HungarianAlgorithm::Solve(vector<vector<double>>& DistMatrix, vector<int>& Assignment)
{
	unsigned int nRows = DistMatrix.size();
    // std::cout << "Solve 1" << std::endl;
	unsigned int nCols = DistMatrix[0].size();
    // std::cout << "Solve 2" << std::endl;
	double *distMatrixIn = new double[nRows * nCols];
	int *assignment = new int[nRows];
	double cost = 0.0;
    // std::cout << "Solve 3" << std::endl;
	// Fill in the distMatrixIn. Mind the index is "i + nRows * j".
	// Here the cost matrix of size MxN is defined as a double precision array of N*M elements. 
	// In the solving functions matrices are seen to be saved MATLAB-internally in row-order.
	// (i.e. the matrix [1 2; 3 4] will be stored as a vector [1 3 2 4], NOT [1 2 3 4]).
	for (unsigned int i = 0; i < nRows; i++)
		for (unsigned int j = 0; j < nCols; j++)
			distMatrixIn[i + nRows * j] = DistMatrix[i][j];
	
    // std::cout << "Solve 4" << std::endl;
	// call solving function
	assignmentoptimal(assignment, &cost, distMatrixIn, nRows, nCols);
    // std::cout << "Solve 5" << std::endl;
	Assignment.clear();
	for (unsigned int r = 0; r < nRows; r++)
		Assignment.push_back(assignment[r]);
    // std::cout << "Solve 6" << std::endl;
	delete[] distMatrixIn;
	delete[] assignment;
	return cost;
}

//********************************************************//
// Solve optimal solution for assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
//********************************************************//
void HungarianAlgorithm::assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
	double *distMatrix, *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;
	bool *coveredColumns, *coveredRows, *starMatrix, *newStarMatrix, *primeMatrix;
	int nOfElements, minDim, row, col;

	/* initialization */
	*cost = 0;
	for (row = 0; row<nOfRows; row++)
		assignment[row] = -1;
    // std::cout << "assignmentoptimal a" << std::endl;
	/* generate working copy of distance Matrix */
	/* check if all matrix elements are positive */
	nOfElements = nOfRows * nOfColumns;
	distMatrix = (double *)malloc(nOfElements * sizeof(double));
	distMatrixEnd = distMatrix + nOfElements;
    // std::cout << "assignmentoptimal b" << std::endl;
	for (row = 0; row<nOfElements; row++)
	{
		value = distMatrixIn[row];
		if (value < 0)
			cerr << "All matrix elements have to be non-negative." << endl;
		distMatrix[row] = value;
	}
    // std::cout << "assignmentoptimal c" << std::endl;

	/* memory allocation */
	coveredColumns = (bool *)calloc(nOfColumns, sizeof(bool));
	coveredRows = (bool *)calloc(nOfRows, sizeof(bool));
	starMatrix = (bool *)calloc(nOfElements, sizeof(bool));
	primeMatrix = (bool *)calloc(nOfElements, sizeof(bool));
	newStarMatrix = (bool *)calloc(nOfElements, sizeof(bool)); /* used in step4 */
    // std::cout << "assignmentoptimal d" << std::endl;
	/* preliminary steps */
	if (nOfRows <= nOfColumns)
	{
		minDim = nOfRows;

		for (row = 0; row<nOfRows; row++)
		{
			/* find the smallest element in the row */
			distMatrixTemp = distMatrix + row;
			minValue = *distMatrixTemp;
			distMatrixTemp += nOfRows;
			while (distMatrixTemp < distMatrixEnd)
			{
				value = *distMatrixTemp;
				if (value < minValue)
					minValue = value;
				distMatrixTemp += nOfRows;
			}

			/* subtract the smallest element from each element of the row */
			distMatrixTemp = distMatrix + row;
			while (distMatrixTemp < distMatrixEnd)
			{
				*distMatrixTemp -= minValue;
				distMatrixTemp += nOfRows;
			}
		}

		/* Steps 1 and 2a */
		for (row = 0; row<nOfRows; row++)
			for (col = 0; col<nOfColumns; col++)
				if (fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)
					if (!coveredColumns[col])
					{
						starMatrix[row + nOfRows*col] = true;
						coveredColumns[col] = true;
						break;
					}
	}
	else /* if(nOfRows > nOfColumns) */
	{
		minDim = nOfColumns;

		for (col = 0; col<nOfColumns; col++)
		{
			/* find the smallest element in the column */
			distMatrixTemp = distMatrix + nOfRows*col;
			columnEnd = distMatrixTemp + nOfRows;

			minValue = *distMatrixTemp++;
			while (distMatrixTemp < columnEnd)
			{
				value = *distMatrixTemp++;
				if (value < minValue)
					minValue = value;
			}

			/* subtract the smallest element from each element of the column */
			distMatrixTemp = distMatrix + nOfRows*col;
			while (distMatrixTemp < columnEnd)
				*distMatrixTemp++ -= minValue;
		}

		/* Steps 1 and 2a */
		for (col = 0; col<nOfColumns; col++)
			for (row = 0; row<nOfRows; row++)
				if (fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)
					if (!coveredRows[row])
					{
						starMatrix[row + nOfRows*col] = true;
						coveredColumns[col] = true;
						coveredRows[row] = true;
						break;
					}
		for (row = 0; row<nOfRows; row++)
			coveredRows[row] = false;

	}
    // std::cout << "assignmentoptimal e" << std::endl;
	/* move to step 2b */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    // std::cout << "assignmentoptimal f" << std::endl;
	/* compute cost and remove invalid assignments */
	computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);
    // std::cout << "assignmentoptimal g" << std::endl;
	/* free allocated memory */
	free(distMatrix);
	free(coveredColumns);
	free(coveredRows);
	free(starMatrix);
	free(primeMatrix);
	free(newStarMatrix);

	return;
}

/********************************************************/
void HungarianAlgorithm::buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns)
{
	int row, col;

	for (row = 0; row<nOfRows; row++)
		for (col = 0; col<nOfColumns; col++)
			if (starMatrix[row + nOfRows*col])
			{
#ifdef ONE_INDEXING
				assignment[row] = col + 1; /* MATLAB-Indexing */
#else
				assignment[row] = col;
#endif
				break;
			}
}

/********************************************************/
void HungarianAlgorithm::computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows)
{
	int row, col;

	for (row = 0; row<nOfRows; row++)
	{
		col = assignment[row];
		if (col >= 0)
			*cost += distMatrix[row + nOfRows*col];
	}
}

/********************************************************/
void HungarianAlgorithm::step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool *starMatrixTemp, *columnEnd;
	int col;
    // std::cout << "step2a a" << std::endl;
	/* cover every column containing a starred zero */
	for (col = 0; col<nOfColumns; col++)
	{
		starMatrixTemp = starMatrix + nOfRows*col;
		columnEnd = starMatrixTemp + nOfRows;
		while (starMatrixTemp < columnEnd){
			if (*starMatrixTemp++)
			{
				coveredColumns[col] = true;
				break;
			}
		}
	}
    // std::cout << "step2a b" << std::endl;
	/* move to step 3 */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
void HungarianAlgorithm::step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	int col, nOfCoveredColumns;
    // std::cout << "step2b 1" << std::endl;
	/* count covered columns */
	nOfCoveredColumns = 0;
	for (col = 0; col<nOfColumns; col++)
		if (coveredColumns[col])
			nOfCoveredColumns++;

    // std::cout << "step2b 2" << std::endl;
	if (nOfCoveredColumns == minDim)
	{
		/* algorithm finished */
        // std::cout << "step2b 3" << std::endl;
		buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
	}
	else
	{
		/* move to step 3 */
        // std::cout << "step2b 4" << std::endl;
		step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
	}
    // std::cout << "step2b 5" << std::endl;
}

/********************************************************/
void HungarianAlgorithm::step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool zerosFound;
	int row, col, starCol;

	zerosFound = true;
	while (zerosFound)
	{
		zerosFound = false;
        // std::cout << "step3 a" << std::endl;
		for (col = 0; col<nOfColumns; col++)
			if (!coveredColumns[col])
                // std::cout << "step3 b" << std::endl;
				for (row = 0; row<nOfRows; row++)
					if ((!coveredRows[row]) && (fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON))
					{
                        // std::cout << "step3 c" << std::endl;
						/* prime zero */
						primeMatrix[row + nOfRows*col] = true;
                        // std::cout << "step3 d" << std::endl;
						/* find starred zero in current row */
						for (starCol = 0; starCol<nOfColumns; starCol++)
							if (starMatrix[row + nOfRows*starCol])
								break;
                        // std::cout << "step3 e" << std::endl;
						if (starCol == nOfColumns) /* no starred zero found */
						{
							/* move to step 4 */
                            // std::cout << "step3 f" << std::endl;
							step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
							return;
						}
						else
						{
                            // std::cout << "step3 g" << std::endl;
							coveredRows[row] = true;
							coveredColumns[starCol] = false;
							zerosFound = true;
							break;
						}
					}
	}
	/* move to step 5 */
    // std::cout << "step3 h" << std::endl;
	step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    // std::cout << "step3 i" << std::endl;
}

/********************************************************/
void HungarianAlgorithm::step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
	int n, starRow, starCol, primeRow, primeCol;
	int nOfElements = nOfRows*nOfColumns;
    // std::cout << "step4 1" << std::endl;
	/* generate temporary copy of starMatrix */
	for (n = 0; n<nOfElements; n++)
		newStarMatrix[n] = starMatrix[n];

	/* star current zero */
	newStarMatrix[row + nOfRows*col] = true;
    // std::cout << "step4 2" << std::endl;
	/* find starred zero in current column */
	starCol = col;
	for (starRow = 0; starRow<nOfRows; starRow++)
		if (starMatrix[starRow + nOfRows*starCol])
			break;
    // std::cout << "step4 3" << std::endl;
	while (starRow<nOfRows)
	{
		/* unstar the starred zero */
		newStarMatrix[starRow + nOfRows*starCol] = false;

		/* find primed zero in current row */
		primeRow = starRow;
		for (primeCol = 0; primeCol<nOfColumns; primeCol++)
			if (primeMatrix[primeRow + nOfRows*primeCol])
				break;

		/* star the primed zero */
		newStarMatrix[primeRow + nOfRows*primeCol] = true;

		/* find starred zero in current column */
		starCol = primeCol;
		for (starRow = 0; starRow<nOfRows; starRow++)
			if (starMatrix[starRow + nOfRows*starCol])
				break;
	}
    // std::cout << "step4 4" << std::endl;
	/* use temporary copy as new starMatrix */
	/* delete all primes, uncover all rows */
	for (n = 0; n<nOfElements; n++)
	{
		primeMatrix[n] = false;
		starMatrix[n] = newStarMatrix[n];
	}
    // std::cout << "step4 5" << std::endl;
	for (n = 0; n<nOfRows; n++)
		coveredRows[n] = false;
    // std::cout << "step4 6" << std::endl;
	/* move to step 2a */
	step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    // std::cout << "step4 7" << std::endl;
}

/********************************************************/
void HungarianAlgorithm::step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	double h, value;
	int row, col;

	/* find smallest uncovered element h */
	h = DBL_MAX;
	for (row = 0; row<nOfRows; row++)
		if (!coveredRows[row])
			for (col = 0; col<nOfColumns; col++)
				if (!coveredColumns[col])
				{
					value = distMatrix[row + nOfRows*col];
					if (value < h)
						h = value;
				}

	/* add h to each covered row */
	for (row = 0; row<nOfRows; row++)
		if (coveredRows[row])
			for (col = 0; col<nOfColumns; col++)
				distMatrix[row + nOfRows*col] += h;

	/* subtract h from each uncovered column */
	for (col = 0; col<nOfColumns; col++)
		if (!coveredColumns[col])
			for (row = 0; row<nOfRows; row++)
				distMatrix[row + nOfRows*col] -= h;

	/* move to step 3 */
	step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

///////////////////////////////////////////////////////////////////////////////
class SORT{
    public:
    SORT();
    typedef std::vector<pcl::PointCloud<pcl::PointXYZI> > Ring;
    typedef std::vector<Ring>                     Zone;
    
    private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher colorize_lidar_pub_;

    // GPF parameter
    int sensor_mode_ = 32;
    double sensor_height_ = 2.2;
    int num_min_pts_ = 10;
    int num_seg_ = 1;
    int num_iter_ = 2;
    int num_lpr_ = 30;
    double th_seeds_ = 0.1;
    double th_dist_ = 0.1;
    float d_;
    Eigen::MatrixXf normal_;
    float th_dist_d_;

    // Patchwork parameter
    double max_range_ = 100.0;
    double min_range_ = 3.2;
    double uprightness_thr_ = 0.866;
    double min_range_z2_ = 7.7;
    double min_range_z3_ = 27.1;
    double min_range_z4_ = 36.6;
    double min_range_z5_ = 70.0;
    int num_zones_ = 5;
    std::vector<int> num_sectors_each_zone_ = {32,64,54,32,16};
    std::vector<int> num_rings_each_zone_ = {4,6,6,4,4};
    std::vector<double> sector_sizes_ = {2 * M_PI / num_sectors_each_zone_.at(0), 2 * M_PI / num_sectors_each_zone_.at(1),
                                        2 * M_PI / num_sectors_each_zone_.at(2),
                                        2 * M_PI / num_sectors_each_zone_.at(3)};
    std::vector<double> ring_sizes_ = {(min_range_z2_ - min_range_) / num_rings_each_zone_.at(0),
                                      (min_range_z3_ - min_range_z2_) / num_rings_each_zone_.at(1),
                                      (min_range_z4_ - min_range_z3_) / num_rings_each_zone_.at(2),
                                      (max_range_ - min_range_z4_) / num_rings_each_zone_.at(3)};
    std::vector<double> min_ranges_ = {min_range_, min_range_z2_, min_range_z3_, min_range_z4_, min_range_z5_};
    std::vector<Zone> ConcetricZoneModel_;

    pcl::PointCloud<pcl::PointXYZI> regionwise_ground_;
    pcl::PointCloud<pcl::PointXYZI> regionwise_nonground_;

    // DBSCAN parameter
    std::vector<Point> Points_;
    double epsilon_ = 2.4; // 2.2
    int min_pts_ = 8; // 8
    int color_arr_[6][3] =
    {
        {255,0,0},
        {0,255,0},
        {0,0,255},
        {255,255,0},
        {255,0,255},
        {0,255,255}
    };
    int cluster_size_ = 0;

    // SORT parameter
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cluster_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> pre_cluster_;
    std::vector<TrackingBox> detData_;
    std::vector<TrackingBox> detCurData_;
    std::vector<KalmanTracker> trackers_;
    int frame_ = 1;


    // Functions
    void RotateZ(double degree, pcl::PointCloud<pcl::PointXYZI>& laserCloudIn, pcl::PointCloud<pcl::PointXYZI>& laserCloudTurn);
    void RemoveShip(const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTurn, const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTurn_pass);
    double XY2Theta(const double &x, const double &y);
    double XY2Radius(const double &x, const double &y);
    void GetCZM(void);
    void InitZone(Zone &z, int num_sectors, int num_rings);
    void FlushPatchesInZone(Zone &patches, int num_sectors, int num_rings);
    void EstimatePlane(void);
    void ExtractInitialSeeds(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI>& p_sorted, pcl::PointCloud<pcl::PointXYZI> &init_seeds);
    void PC2CZM(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm);
    void ExtractPiecewiseground(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &dst, pcl::PointCloud<pcl::PointXYZI> &non_ground_dst);
    void EstimateGround(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, pcl::PointCloud<pcl::PointXYZI> &cloud_out, pcl::PointCloud<pcl::PointXYZI> &cloud_nonground);
    
    void ConvertPC(const pcl::PointCloud<pcl::PointXYZI> laserCloudIn);
    void Colorize(const std::vector<Point> points, pcl::PointCloud<pcl::PointXYZRGB> &color_laser);
    std::vector<int> CalculateCluster(const pcl::KdTreeFLANN<pcl::PointXYZI> kdtree, Point &point);
    int ExpandCluster(const pcl::KdTreeFLANN<pcl::PointXYZI> kdtree, Point &cur_point, int cluster_ID);
    
    void GetCluster(void);
    void MakeBbox(void);
    void TestSORT(void);
    void SORTCallback(const sensor_msgs::PointCloud2ConstPtr &input_lidar_msg);
};

SORT::SORT() : nh_ ("~"){
    lidar_sub_ = nh_.subscribe("/lidar_data",1, &SORT::SORTCallback, this);
    colorize_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dbscan_cloud", 1);
}

void SORT::RotateZ(double degree, pcl::PointCloud<pcl::PointXYZI>& laserCloudIn, pcl::PointCloud<pcl::PointXYZI>& laserCloudTurn){
    double rad = degree * M_PI/180;
    Eigen::Affine3f rotation_z = Eigen::Affine3f::Identity(); // 3x3???????????? I??? ?????????
    rotation_z.rotate(Eigen::AngleAxisf(rad, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(laserCloudIn, laserCloudTurn, rotation_z);
}

void SORT::RemoveShip(const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTurn, const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTurn_pass){
    pcl::PointCloud<pcl::PointXYZI>::Ptr center(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_x.setInputCloud(laserCloudTurn);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-8.0, 4.3); // -8.0, 4.3 ,  -1.1, 3.0
    pass_x.filter(*center); // x ?????? ??? ????????? ??????
    pass_x.setFilterLimitsNegative(true); // true = ?????? ???
    pass_x.filter(*laserCloudTurn_pass); // x ?????? ??? ????????? ??????
    pcl::PassThrough<pcl::PointXYZI> pass_y;
    pass_y.setInputCloud(center);  // x ?????? ????????? y ?????? ??? ??????
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-2.0, 2.0); // -2.0, 2.0  ,  -1.1, 1.1
    pass_y.setFilterLimitsNegative(true);
    pass_y.filter(*output);
    *laserCloudTurn_pass += *output; // ??????
}

double SORT::XY2Theta(const double &x, const double &y){
    auto atan_value = atan2(y,x);
    return atan_value > 0 ? atan_value : atan_value + 2*M_PI;
}

double SORT::XY2Radius(const double &x, const double &y){
    return sqrt(pow(x, 2) + pow(y, 2));
}

void SORT::GetCZM(void){
    for(int iter = 0; iter < num_zones_; ++iter){
        Zone z;
        InitZone(z, num_sectors_each_zone_.at(iter), num_rings_each_zone_.at(iter));
        ConcetricZoneModel_.push_back(z);
    }
}

void SORT::InitZone(Zone &z, int num_sectors, int num_rings){
    z.clear();
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(1000);
    Ring ring;
    for(int i=0; i<num_sectors; i++){
        ring.emplace_back(cloud);
    }
    for(int j=0; j<num_rings; j++){
        z.emplace_back(ring);
    }
}

void SORT::FlushPatchesInZone(Zone &patches, int num_sectors, int num_rings){
    for (int i = 0; i < num_sectors; i++) {
        for (int j = 0; j < num_rings; j++) {
            if (!patches[j][i].points.empty()) patches[j][i].points.clear();
        }
    }
}

void SORT::EstimatePlane(void){
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    normal_ = (svd.matrixU().col(2));
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    d_ = -(normal_.transpose()*seeds_mean)(0.0);
    th_dist_d_ = th_dist_ - d_;
}

void SORT::ExtractInitialSeeds(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI>& p_sorted, pcl::PointCloud<pcl::PointXYZI> &init_seeds){
    init_seeds.points.clear();
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    int init_idx = 0;
    // Calculate the mean height value.
    for (int i          = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double   lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}

void SORT::PC2CZM(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm){
    for (auto const &pt : src.points) {
        int    ring_idx, sector_idx;
        double r = XY2Radius(pt.x, pt.y);
        if ((r <= max_range_) && (r > min_range_)) {
            double theta = XY2Theta(pt.x, pt.y);
            if (r < min_range_z2_) { // In First rings
                ring_idx   = std::min(static_cast<int>(((r - min_range_) / ring_sizes_[0])), num_rings_each_zone_[0] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[0])), num_sectors_each_zone_[0] - 1);
                czm[0][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z3_) {
                ring_idx   = std::min(static_cast<int>(((r - min_range_z2_) / ring_sizes_[1])), num_rings_each_zone_[1] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[1])), num_sectors_each_zone_[1] - 1);
                czm[1][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z4_) {
                ring_idx   = std::min(static_cast<int>(((r - min_range_z3_) / ring_sizes_[2])), num_rings_each_zone_[2] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[2])), num_sectors_each_zone_[2] - 1);
                czm[2][ring_idx][sector_idx].points.emplace_back(pt);
            } else { // Far!
                ring_idx   = std::min(static_cast<int>(((r - min_range_z4_) / ring_sizes_[3])), num_rings_each_zone_[3] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[3])), num_sectors_each_zone_[3] - 1);
                czm[3][ring_idx][sector_idx].points.emplace_back(pt);
            }
        }
    }
}

void SORT::ExtractPiecewiseground(const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &dst, pcl::PointCloud<pcl::PointXYZI> &non_ground_dst){
    // 0. Initialization
    if (!g_ground_pc->empty()) g_ground_pc->clear();
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();

    // 1. set seeds!
    ExtractInitialSeeds(zone_idx, src, *g_ground_pc);

    // 2. Extract ground
    for (int i = 0; i < num_iter_; i++){
        EstimatePlane();
        g_ground_pc->clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(src.points.size(), 3);
        int j=0;
        for (auto       &p:src.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int        r      = 0; r < result.rows(); r++) {
            if (i < num_iter_ - 1) {
                if (result[r] < th_dist_d_) {
                    g_ground_pc->points.push_back(src[r]);
                }
            } else { // Final stage
                if (result[r] < th_dist_d_) {
                    dst.points.push_back(src[r]);
                } else {
                    if (i == num_iter_ - 1) {
                        non_ground_dst.push_back(src[r]);
                    }
                }
            }
        }
    }
}

void SORT::EstimateGround(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, pcl::PointCloud<pcl::PointXYZI> &cloud_out, pcl::PointCloud<pcl::PointXYZI> &cloud_nonground){
    // Ground estimation for each patch
    GetCZM();
    for(int k=0; k<num_zones_; ++k){
        FlushPatchesInZone(ConcetricZoneModel_[k], num_sectors_each_zone_[k], num_rings_each_zone_[k]);
    }
    PC2CZM(cloud_in, ConcetricZoneModel_);

    cloud_out.clear();
    cloud_nonground.clear();

    int concentric_idx = 0;
    for(int k = 0; k < num_zones_; ++k){
        auto zone = ConcetricZoneModel_[k];
        for(uint16_t ring_idx = 0; ring_idx < num_rings_each_zone_[k]; ++ring_idx){
            for(uint16_t sector_idx = 0; sector_idx < num_sectors_each_zone_[k]; ++sector_idx){
                if(zone[ring_idx][sector_idx].points.size() > num_min_pts_){
                    sort(zone[ring_idx][sector_idx].points.begin(), zone[ring_idx][sector_idx].end(), point_compare);
                    ExtractPiecewiseground(k, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);

                    const double ground_z_vec = abs(normal_(2,0));

                    if(ground_z_vec < uprightness_thr_){ // All points are rejected
                        cloud_out += regionwise_ground_;
                        cloud_nonground += regionwise_nonground_;
                    }
                    else{ // satisfy uprightness
                        cloud_out += regionwise_ground_;
                        cloud_nonground += regionwise_nonground_;
                    }
                }
            }
            ++concentric_idx;
        }
    }
}

void SORT::ConvertPC(const pcl::PointCloud<pcl::PointXYZI> laserCloudIn){
  for(int i = 0; i < laserCloudIn.points.size(); i++){
    if(!std::isnan(laserCloudIn.points[i].x) && !std::isnan(laserCloudIn.points[i].y) && !std::isnan(laserCloudIn.points[i].z)){
      Point temp;
      temp.p.x = laserCloudIn.points[i].x;
      temp.p.y = laserCloudIn.points[i].y;
      temp.p.z = laserCloudIn.points[i].z;
      temp.p.intensity = laserCloudIn.points[i].intensity;
      temp.clusterID = -1;
      Points_.push_back(temp);
    }
  }
}

void SORT::Colorize(const std::vector<Point> points, pcl::PointCloud<pcl::PointXYZRGB> &color_laser){
  int N = points.size();
  color_laser.clear();
  pcl::PointXYZRGB temp;
  for(int i=0; i<N; ++i){
    if(points[i].clusterID > 0){
      const auto &pt = points[i].p;
      temp.x = pt.x;
      temp.y = pt.y;
      temp.z = pt.z;
      temp.r = color_arr_[points[i].clusterID - 1][0];
      temp.g = color_arr_[points[i].clusterID - 1][1];
      temp.b = color_arr_[points[i].clusterID - 1][2];
      color_laser.push_back(temp);
    }
  }
}

std::vector<int> SORT::CalculateCluster(const pcl::KdTreeFLANN<pcl::PointXYZI> kdtree, Point &point){
  std::vector<int> indices;
  std::vector<float> sqr_dists;
  kdtree.radiusSearch(point.p, epsilon_, indices, sqr_dists);
  return indices;
}

int SORT::ExpandCluster(const pcl::KdTreeFLANN<pcl::PointXYZI> kdtree, Point &cur_point, int cluster_ID){
  std::vector<int> cluster_seeds = CalculateCluster(kdtree, cur_point);
  if(cluster_seeds.size() < min_pts_){
    cur_point.clusterID = -2;
    // std::cout << "(" << cur_point.p.x << ", " << cur_point.p.y << ", " << cur_point.p.z << ", " << cluster_seeds.size() << ")";
    return -3; // -3 = fail
  }
  else{
    cur_point.clusterID = cluster_ID;
    int index = 0;
    int indexCorePoint = 0;
    std::vector<int>::iterator iter_seeds;
    for(iter_seeds = cluster_seeds.begin(); iter_seeds != cluster_seeds.end(); ++iter_seeds){
      if(Points_.at(*iter_seeds).p.x == cur_point.p.x && Points_.at(*iter_seeds).p.y == cur_point.p.y && Points_.at(*iter_seeds).p.z == cur_point.p.z){
        indexCorePoint = index;
      }
      index++;
    }
    cluster_seeds.erase(cluster_seeds.begin() + indexCorePoint);

    for(std::vector<int>::size_type i = 0, n = cluster_seeds.size(); i < n; ++i){
      std::vector<int> cluster_neighbors = CalculateCluster(kdtree, Points_.at(cluster_seeds[i]));
      if(cluster_neighbors.size() >= min_pts_){
        std::vector<int>::iterator iter_neighbors;
        for(iter_neighbors = cluster_neighbors.begin(); iter_neighbors != cluster_neighbors.end(); ++iter_neighbors){
          if(Points_.at(*iter_neighbors).clusterID == -1 || Points_.at(*iter_neighbors).clusterID == -2){
            cluster_seeds.push_back(*iter_neighbors);
            n = cluster_seeds.size();
          }
          Points_.at(*iter_neighbors).clusterID = cluster_ID;
        }
      }
    }
    // std::cout << "(" << cur_point.p.x << ", " << cur_point.p.y << ", " << cur_point.p.z << ", " << cluster_seeds.size() << ")";
    return 0; // 0 = success
  }
}

void SORT::GetCluster(void){    
    cluster_.resize(cluster_size_);
    int n = Points_.size();
    std::cout << "Total Points size : " << Points_.size() << std::endl;
    int id = 1;
    pcl::PointCloud<pcl::PointXYZI> color_cluster;
    for(int k = 0; k < cluster_size_; k++){
        for(int i = 0; i < n; i++){
            if(Points_[i].clusterID == id){
                pcl::PointXYZI temp = Points_[i].p;
                cluster_[id - 1].push_back(temp);
            }
        }
        id++;
    }

    // for(int j = 0; j < cluster_.size(); j++){
    //     std::cout << "cluster " << j+1 << " size : " << cluster_[j].points.size()  << std::endl;
    // }        

}

void SORT::MakeBbox(void){
    for(int i = 0; i < cluster_.size(); i++){
        double sum_x = 0.0;
        double sum_y = 0.0;
        double mean_x = 0.0;
        double mean_y = 0.0;
        double height = 0.0; 
        double width = 0.0;
        // std::cout << sum_x << ", " << sum_y << ", " << mean_x << ", " << mean_y << ", " << height << ", " << width << std::endl;
        TrackingBox tb;
        // std::cout << "//////////////////////////////////////////////" << std::endl;
        // std::cout << "cluster " << i+1 << " size : " << cluster_[i].points.size()  << std::endl;

        std::vector<double> vel_x, vel_y;        
        for(int j = 0; j < cluster_[i].points.size(); j++){
            vel_x.push_back(cluster_[i].points[j].x);
            vel_y.push_back(cluster_[i].points[j].y);
            sum_x += vel_x[j];
            sum_y += vel_y[j];
        }
        mean_x = sum_x / cluster_[i].points.size();
        mean_y = sum_y / cluster_[i].points.size();
        // std::cout << "sum " << i+1 << " : " << sum_x << ", " << sum_y << std::endl;

        std::vector<double> dist_x_arr, dist_y_arr;
        for(int j = 0; j < cluster_[i].points.size(); j++){
            double dist_x = abs(mean_x - cluster_[i].points[j].x);
            double dist_y = abs(mean_y - cluster_[i].points[j].y);
            dist_x_arr.push_back(dist_x);
            dist_y_arr.push_back(dist_y);
        }
        std::sort(dist_x_arr.begin(), dist_x_arr.end());
        std::sort(dist_y_arr.begin(), dist_y_arr.end());
        height = dist_x_arr[cluster_[i].points.size() - 1];
        width = dist_y_arr[cluster_[i].points.size() - 1];

        float cx = 200 - mean_y;
        float cy = 200 - mean_x;
        float w = 2 * width;
        if(w < 5.0) w = 5.0;
        float h = 2 * height;
        if(h < 5.0) h = 5.0;

        std::cout << std::setprecision(11) << "cluster " << i + 1 << " tpx : " << cx - w / 2 << ", tpy : " << cy - h / 2 << ", height : " << h << ", width : " << w << std::endl;
        tb.box = cv::Rect_<float>((cx - w / 2), (cy - h / 2), w, h);
        // tb.box = cv::Rect_<float>(cx, cy, w, h);
        tb.frame = frame_;
        tb.id = -1;
        detData_.push_back(tb);
        detCurData_.push_back(tb);

        double b = color_arr_[i][2];
        double g = color_arr_[i][1];
        double r = color_arr_[i][0];
        cv::Mat rec_img = cv::Mat::zeros(400,400, CV_8U);
        cv::resize(rec_img, rec_img, cv::Size(400,400));
        cv::rectangle(rec_img, cv::Rect((cx - w / 2), (cy - h / 2), w, h), cv::Scalar(b,g,r), 1, 8, 0);
        cv::imwrite("./test_img.bmp", rec_img);        
        /****************************************************************************************************/
        dist_x_arr.clear();
        dist_y_arr.clear();
        vel_x.clear();
        vel_y.clear();
    }
}

void SORT::TestSORT(void){
    if(cluster_.size() > 0){
        // 1. make tracking detection data and push back to detData_.
        MakeBbox();

        // 2. group detData_ by frame
        int maxFrame = 0;
        for(auto tb : detData_){
            if(maxFrame < tb.frame) maxFrame = tb.frame;
        }
        // std::cout << "maxFrame : " << maxFrame << std::endl;

        std::vector<std::vector<TrackingBox>> detFrameData;
        std::vector<TrackingBox> tempVec;
        for(int i = 0; i < maxFrame; i++){
            for(auto tb : detData_){
                if(tb.frame = i + 1) 
                    tempVec.push_back(tb);
            }
            detFrameData.push_back(tempVec);
            tempVec.clear();
        }
        // std::cout << "detData_ size : " << detData_.size() << std::endl;
        // std::cout << "detCurData_ size : " << detCurData_.size() << std::endl;
        // std::cout << "detFrameData size : " << detFrameData.size() << std::endl;

        // 3. update across frames
        int frame_count = 0;
        int max_age = 1;
        int min_hits = 3;
        double iouThreshold = 0.3;
        KalmanTracker::kf_count = 0; // tracking id relies on this, so we have to reset it in each seq.

        // variables used in the for-loop
        std::vector<cv::Rect_<float>> predictedBoxes;
        std::vector<std::vector<double>> iouMatrix;
        std::vector<int> assignment;
        set<int> unmatchedDetections;
        set<int> unmatchedTrajectories;
        set<int> allItems;
        set<int> matchedItems;
        std::vector<cv::Point> matchedPairs;
        std::vector<TrackingBox> frameTrackingResult;
        unsigned int trkNum = 0;
        unsigned int detNum = 0;

        //////////////////////////////////////////////
        // main
        if(detCurData_.size() > 0){
            // Initialize
            if(trackers_.size() == 0){
                // initialize kalman trackers using first detections.
                for (unsigned int i = 0; i < detCurData_.size(); i++){
                    KalmanTracker trk = KalmanTracker(detCurData_[i].box);
                    trackers_.push_back(trk);
                }
            }
            ///////////////////////////////////////
            // 3.1. get predicted locations from existing trackers
            std::cout << "//////////////////////////////////////////////" << std::endl;
            std::cout << "trackers size : " << trackers_.size() << std::endl;
            predictedBoxes.clear();
            for(int i = 0; i < trackers_.size(); i++){
                cv::Rect_<float> pBox = trackers_[i].predict();
                std::cout << "x, y : " << pBox.x << ", " << pBox.y << std::endl;
                predictedBoxes.push_back(pBox);
            }

            ///////////////////////////////////////
            // 3.2. associate detections to tracked object (both represented as bounding boxes)
            trkNum = predictedBoxes.size();
            detNum = detCurData_.size();
            std::cout << "//////////////////////////////////////////////" << std::endl;
            std::cout << "trk / det : " << trkNum << ", " << detNum << std::endl;

            iouMatrix.clear();
            iouMatrix.resize(trkNum, std::vector<double>(detNum, 0));

            for(unsigned int i = 0; i < trkNum; i++){
                for(unsigned int j = 0; j < detNum; j++){
                    iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detCurData_[j].box);
                    // iouMatrix[i][j] = 1 - 0.5;
                }
            }

            // solve the assignment problem using hungarian algorithm.
		    // the resulting assignment is [track(prediction) : detection], with len=preNum
            HungarianAlgorithm HungAlgo;
            assignment.clear();
            HungAlgo.Solve(iouMatrix, assignment);

            // find matches, unmatched_detections and unmatched_predictions
            unmatchedTrajectories.clear();
            unmatchedDetections.clear();
            allItems.clear();
            matchedItems.clear();

            if (detNum > trkNum) //	there are unmatched detections
            {
                for (unsigned int n = 0; n < detNum; n++)
                    allItems.insert(n);

                for (unsigned int i = 0; i < trkNum; ++i)
                    matchedItems.insert(assignment[i]);

                set_difference(allItems.begin(), allItems.end(),
                    matchedItems.begin(), matchedItems.end(),
                    insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
            }
            else if (detNum < trkNum) // there are unmatched trajectory/predictions
            {
                for (unsigned int i = 0; i < trkNum; ++i)
                    if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
                        unmatchedTrajectories.insert(i);
            }
            else
                ;
            // filter out matched with low IOU
            matchedPairs.clear();
            for (unsigned int i = 0; i < trkNum; ++i)
            {
                if (assignment[i] == -1) // pass over invalid values
                    continue;
                if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
                {
                    unmatchedTrajectories.insert(i);
                    unmatchedDetections.insert(assignment[i]);
                }
                else
                    matchedPairs.push_back(cv::Point(i, assignment[i]));
            }
            ///////////////////////////////////////
		    // 3.3. updating trackers
            // update matched trackers with assigned detections.
            // each prediction is corresponding to a tracker
            int detIdx, trkIdx;
            for (unsigned int i = 0; i < matchedPairs.size(); i++)
            {
                trkIdx = matchedPairs[i].x;
                detIdx = matchedPairs[i].y;
                trackers_[trkIdx].update(detCurData_[detIdx].box);
            }

            // create and initialise new trackers for unmatched detections
            for (auto umd : unmatchedDetections)
            {
                KalmanTracker tracker = KalmanTracker(detCurData_[umd].box);
                trackers_.push_back(tracker);
		    }

            // get trackers' output
            frameTrackingResult.clear();
            for (auto it = trackers_.begin(); it != trackers_.end();)
            {
                if (((*it).m_time_since_update < 1) &&
                    ((*it).m_hit_streak >= min_hits || frame_count <= min_hits))
                {
                    TrackingBox res; 
                    res.box = (*it).get_state();
                    res.id = (*it).m_id + 1;
                    res.frame = frame_count;
                    frameTrackingResult.push_back(res);
                    it++;
                }
                else
                    it++;

                // remove dead tracklet
                if (it != trackers_.end() && (*it).m_time_since_update > max_age)
                    it = trackers_.erase(it);
            }

        }
        /****************************************************************************************************/
    }
}


void SORT::SORTCallback(const sensor_msgs::PointCloud2ConstPtr &input_lidar_msg){
    if(input_lidar_msg->data.size() > 0){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_lidar_msg, *cloud_in);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_turn(new pcl::PointCloud<pcl::PointXYZI>);
        RotateZ(180, *cloud_in, *cloud_turn);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZI>);
        RemoveShip(cloud_turn, cloud_pass);
        EstimateGround(*cloud_pass, *g_ground_pc, *g_not_ground_pc);
        /****************************************************************************************************/
        ConvertPC(*g_not_ground_pc);
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(g_not_ground_pc);
        int clusterID = 1;
        std::vector<Point>::iterator iter;
        for(iter = Points_.begin(); iter != Points_.end(); ++iter){
            if(iter -> clusterID == -1){
                if(ExpandCluster(kdtree, *iter, clusterID) != -3){
                    clusterID += 1;
                }
            }
        }
        cluster_size_ = clusterID - 1;
        std::cout << "-------------------------------------------------------" << std::endl;
        std::cout << "frame " << frame_ << std::endl;
        GetCluster();
        std::cout << "num of cluster : " << cluster_.size() << std::endl;
        /****************************************************************************************************/
        TestSORT();
        std::cout << "-------------------------------------------------------" << std::endl;
        /****************************************************************************************************/
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
        Colorize(Points_, *pc_colored);
        sensor_msgs::PointCloud2 color_lidar_msg;
        pcl::toROSMsg(*pc_colored, color_lidar_msg);  
        color_lidar_msg.header.frame_id = input_lidar_msg->header.frame_id;
        colorize_lidar_pub_.publish(color_lidar_msg);
    }
    /******************************************************************************************/
    Points_.clear();
    cluster_.clear();
    detCurData_.clear();    
    frame_++;
    /******************************************************************************************/
}

int main(int argc, char** argv){
    ros::init(argc, argv, "SORT");
    SORT node;
    ros::spin();
    return 0;
}