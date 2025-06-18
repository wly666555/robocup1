/**
 * @file locator.h
 * @brief It is carried out using the particle filter algorithm and locates (itself) through the landmark points on the pitch.
 */
#pragma once

#include <Eigen/Core>
#include <cstdlib> // for srand and rand
#include <ctime>   // for time
#include <limits>
#include <cmath>
#include <chrono>

#include "types.h"

// #define EPSILON 1e-5

using namespace std;
namespace chr = std::chrono;

struct PoseBox2D
{
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double thetamin;
	double thetamax;
};

struct FieldMarker
{
	char type;			// L|T|X|P, representing different types of landmark points, where P represents the penalty mark.
	double x, y; // The position of the landmark point (in meters).
	double confidence;	// The recognition confidence level.
};

// The positioning result.
struct LocateResult
{
	bool success = false;
	// 0: Success
	// 1: Failure to generate new particles (quantity is 0)
	// 2: The residual error after convergence is unreasonable
	// 3: Not converged
	// 4: The number of Markers is insufficient
	// 5: The probabilities of all particles are too low
	// -1 represents the initial state
	int code = -1;
	double residual = 0; // The average residual error.
	Pose2D pose;
	int msecs = 0; // The time consumed for positioning.
};

/**
 * @class Locator
 * @brief Use particle filtering to position the robot.
 */
class Locator
{
public:
	// Parameters
	double convergeTolerance = 0.2; // When the x, y, and theta ranges of all hypos are less than this value, it is considered that convergence has been achieved.
	double residualTolerance = 0.4; // If the average residual for each marker is greater than this value, the converged position is considered unreasonable.
	double maxIteration = 20;		// The maximum number of iterations.
	double muOffset = 2.0;			// It is approximately considered that the mu of the residual distribution is min(residuals) - muOffset * std(residuals).
	double numShrinkRatio = 0.85;	// During each resampling, the number of particles is this proportion of that in the previous time.
	double offsetShrinkRatio = 0.8; // During each resampling, the offsets of x, y, and theta are reduced to this proportion compared to the previous time.
	double minMarkerCnt = 3;		// The minimum number of Markers required.

	// Data storage
	vector<FieldMarker> fieldMarkers;
	FieldDimensions fieldDimensions;
	Eigen::ArrayXXd hypos;				  // An n * 5 matrix used to store the calculation results, where n is the number of hypotheses. Columns 0, 1, and 2 are the hypothesized Pose (x, y, theta), column 3 is the residual, column 4 is the normalized probability, and column 5 is the cumulative sum of the normalized probabilities (the sum from row 0 to the current row).
	PoseBox2D constraints;				  // The range constraint for positioning.
	double offsetX, offsetY, offsetTheta; // When generating new particles, the random offset range from the particles in the previous round. That is, the new x is randomly within the range of [-offsetX, offsetX] of the old x.
	Pose2D bestPose;					  // The best hypothesized position for each positioning.
	double bestResidual;				  // The minimum residual for each positioning.

	void init(FieldDimensions fd, int minMarkerCnt = 4, double residualTolerance = 0.4, double muOffsetParam = 2.0);

	/**
	 * @brief Generate the positions in the pitch coordinate system of all the landmark points on the pitch based on the pitch dimension information.
	 *
	 * @param fieldDimensions FieldDimensions, the pitch dimension information.
	 *
	 */
	void calcFieldMarkers(FieldDimensions fd);

	/**
	 * @brief Calculate the Pose of the robot on the pitch and return the int code.
	 *
	 * @param fieldDimensions FieldDimensions, the pitch dimension information.
	 * @param markers_r vector<FieldMarker>, the positions of the pitch landmark points in the robot coordinate system obtained through vision.
	 * @param constraints PoseBox2D, the prior constraint conditions for positioning. The stricter the constraint conditions are, the faster the positioning will be.
	 * @param pose &Pose2D, the output result of the positioning.
	 *
	 * @return int, 0: Success; 1: Failure to generate new particles (quantity is 0); 2: The residual error after convergence is unreasonable; 3: Not converged; 4: The number of Markers is insufficient; 5: The probabilities of all particles are too low.
	 *
	 */
	int locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraints, Pose2D &pose, double &residual, int numParticles = 200, double offsetX = 2.0, double offsetY = 2.0, double offsetTheta = M_PI / 4);

	/**
	 * @brief Calculate the Pose of the robot on the pitch and return the struct result.
	 *
	 * @param fieldDimensions FieldDimensions, the pitch dimension information.
	 * @param markers_r vector<FieldMarker>, the positions of the pitch landmark points in the robot coordinate system obtained through vision.
	 * @param constraints PoseBox2D, the prior constraint conditions for positioning. The stricter the constraint conditions are, the faster the positioning will be.
	 *
	 * @return LocateResult
	 *         success: bool
	 *         code: int, 0: Success; 1: Failure to generate new particles (quantity is 0); 2: The residual error after convergence is unreasonable; 3: Not converged; 4: The number of Markers is insufficient; 5: The probabilities of all particles are too low.
	 *         residual: double, the residual error.
	 *         Pose2D: the positioning result.
	 */
	LocateResult locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraints, int numParticles = 200, double offsetX = 2.0, double offsetY = 2.0, double offsetTheta = M_PI / 4);

	/**
	 * @brief Generate initial particles.
	 *
	 * @param constraints PoseBox2D, generate hypothesized Poses only within the range of constraints.
	 * @param num int, specify how many hypotheses to generate.
	 *
	 * @return int, 0 indicates success, and non-zero indicates failure.
	 *
	 */
	int genInitialParticles(int num = 200);

	/**
	 * @brief Resample to generate new particles according to the probabilities.
	 *
	 * @return int, 0 indicates success, and non-zero indicates failure.
	 */
	int genParticles();

	/**
	 * @brief Convert the pitch landmark points observed by the robot from the robot coordinate system to the pitch coordinate system according to the Pose of the robot.
	 *
	 * @param FieldMarker marker
	 * @param Pose2D pose
	 *
	 * @return FieldMarker, a vector containing all the landmark points.
	 *
	 */
	FieldMarker markerToFieldFrame(FieldMarker marker, Pose2D pose);

	/**
	 * @brief Get the minimum distance between an observed marker and all markers on the pitch map.
	 *
	 * @param marker FieldMarker
	 *
	 * @return double The minimum distance.
	 *
	 */
	double minDist(FieldMarker marker);

	/**
	 * @brief Get the Pose offset between an observed marker and the nearest marker on the pitch map.
	 *
	 * @param marker FieldMarker
	 *
	 * @return vector<double> {dx, dy} The offsets of x and y from the nearest marker, with signs. dx = x(map marker) - x(observed marker).
	 *
	 */
	vector<double> getOffset(FieldMarker marker);

	/**
	 * @brief Calculate the fitting residual between a set of observed markers (in the robot coordinate system) and the pitch markers.
	 *
	 * @param markers_r vector<FieldMarker> markers_r The observed markers.
	 * @param pose Pose2D The pose of the robot, which is used to convert markers_r into the pitch coordinate system.
	 *
	 * @return double The residual.
	 *
	 */
	double residual(vector<FieldMarker> markers_r, Pose2D pose);

	/**
	 * @brief Check whether the current hypos have converged.
	 *
	 * @return bool
	 */
	bool isConverged();

	/**
	 * @brief Calculate the corresponding probabilities according to the current position hypotheses and store them in the member probs.
	 *
	 * @return int, 0 indicates success, and non-zero indicates failure.
	 */
	int calcProbs(vector<FieldMarker> markers_r);

	/**
	 * @brief After convergence, recalibrate x and y according to the positions of the markers.
	 *
	 * @return Pose2D, the calibrated Pose.
	 */
	Pose2D finalAdjust(vector<FieldMarker> markers_r, Pose2D pose);

	/**
	 * @brief Gaussian distribution probability density function.
	 *
	 * @param r double The observed value.
	 * @param mu double The mean of the distribution.
	 * @param sigma double The standard deviation of the distribution.
	 *
	 * @return double The probability density.
	 *
	 */
	inline double probDesity(double r, double mu, double sigma)
	{
		if (sigma < 1e-5)
			return 0.0;
		return 1 / sqrt(2 * M_PI * sigma * sigma) * exp(-(r - mu) * (r - mu) / (2 * sigma * sigma));
	};

	/**
	 * @brief Log particles(hypos) to rerun
	 *
	 */
	void logParticles();
};
