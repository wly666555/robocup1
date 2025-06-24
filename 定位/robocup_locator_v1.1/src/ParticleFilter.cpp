#include "ParticleFilter.h"
#include "math.h"

#include "misc.h"
#include "types.h"
#include <iostream>
#include <vector>


void ParticleFilter::init(FieldDimensions fd, int minMarkerCntParam, double residualToleranceParam, double muOffestParam)
{
    fieldDimensions = fd;
    calcFieldMarkers(fd);
    minMarkerCnt = minMarkerCntParam;
    residualTolerance = residualToleranceParam * 2.0;
    muOffset = muOffestParam;
}

void ParticleFilter::calcFieldMarkers(FieldDimensions fd)
{
    // The X mark on the center line.
    fieldMarkers.push_back(FieldMarker{'X', 0.0, -fd.circleRadius});
    fieldMarkers.push_back(FieldMarker{'X', 0.0, fd.circleRadius});

    // Penalty spot
    fieldMarkers.push_back(FieldMarker{'X', fd.length / 2 - fd.penaltyDist, 0.0});
    fieldMarkers.push_back(FieldMarker{'X', -fd.length / 2 + fd.penaltyDist, 0.0});

    // Center of the sideline
    fieldMarkers.push_back(FieldMarker{'T', 0.0, fd.width / 2});
    fieldMarkers.push_back(FieldMarker{'T', 0.0, -fd.width / 2});

    // Penalty area
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.penaltyAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.penaltyAreaWidth / 2});

    // Goal area
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', (fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'L', -(fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', fd.length / 2, -fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, fd.goalAreaWidth / 2});
    fieldMarkers.push_back(FieldMarker{'T', -fd.length / 2, -fd.goalAreaWidth / 2});

    // Four corners of the field
    fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, fd.width / 2});
    fieldMarkers.push_back(FieldMarker{'L', fd.length / 2, -fd.width / 2});
    fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, fd.width / 2});
    fieldMarkers.push_back(FieldMarker{'L', -fd.length / 2, -fd.width / 2});
}

FieldMarker ParticleFilter::markerToFieldFrame(FieldMarker marker_r, Pose2D pose_r2f)
{
    auto [x, y, theta] = pose_r2f;

    Eigen::Matrix3d transform;
    transform << cos(theta), -sin(theta), x,
        sin(theta), cos(theta), y,
        0, 0, 1;

    Eigen::Vector3d point_r;
    point_r << marker_r.x, marker_r.y, 1.0;

    auto point_f = transform * point_r;

    return FieldMarker{marker_r.type, point_f.x(), point_f.y()};
}


int ParticleFilter::genInitialParticles(int num)
{
    unsigned long long seed = chr::duration_cast<std::chrono::milliseconds>(chr::system_clock::now().time_since_epoch()).count();
    srand(seed);
    hypos.resize(num, 6);
    hypos.leftCols(3) = Eigen::ArrayXXd::Random(num, 3);

    auto [xmin, xmax, ymin, ymax, thetamin, thetamax] = constraints;
    hypos.col(0) = hypos.col(0) * (xmax - xmin) / 2 + (xmin + xmax) / 2;
    hypos.col(1) = hypos.col(1) * (ymax - ymin) / 2 + (ymin + ymax) / 2;
    hypos.col(2) = hypos.col(2) * (thetamax - thetamin) / 2 + (thetamin + thetamax) / 2;

    return 0;
}

int ParticleFilter::genParticles()
{
    auto old_hypos = hypos;

    if (old_hypos.rows() <= 0) {
        return 1;
    }

    int num = static_cast<int>(old_hypos.rows() * numShrinkRatio);
    if (num <= 0) {
        return 1;
    }

    try {
        hypos.resize(num, 6);
        hypos.setZero();
    } catch (const std::exception& e) {
        return 1;
    }

    Eigen::ArrayXd rands = (Eigen::ArrayXd::Random(num) + 1) / 2;

    for (int i = 0; i < num && i < rands.size(); i++) {
        double rand = rands(i);
        int j = 0;
        
        while (j < old_hypos.rows() && old_hypos(j, 5) < rand) {
            j++;
        }

        if (j < old_hypos.rows()) {
            hypos.row(i).head(3) = old_hypos.row(j).head(3);
        }
    }

    offsetX *= offsetShrinkRatio;
    offsetY *= offsetShrinkRatio;
    offsetTheta *= offsetShrinkRatio;

    try {
        Eigen::ArrayXXd offsets = Eigen::ArrayXXd::Random(num, 3);
        offsets.col(0) *= offsetX;
        offsets.col(1) *= offsetY;
        offsets.col(2) *= offsetTheta;
        hypos.leftCols(3) += offsets;
    } catch (const std::exception& e) {
        return 1;
    }

    hypos.col(0) = hypos.col(0).cwiseMax(constraints.xmin).cwiseMin(constraints.xmax);
    hypos.col(1) = hypos.col(1).cwiseMax(constraints.ymin).cwiseMin(constraints.ymax);
    hypos.col(2) = hypos.col(2).cwiseMax(constraints.thetamin).cwiseMin(constraints.thetamax);

    return 0;
}


double ParticleFilter::minDist(FieldMarker marker)
{
    double minDist = std::numeric_limits<double>::infinity();
    double dist;
    for (int i = 0; i < fieldMarkers.size(); i++)
    {
        auto target = fieldMarkers[i];
        if (target.type != marker.type)
        {
            continue;
        }
        dist = sqrt(pow((target.x - marker.x), 2.0) + pow((target.y - marker.y), 2.0));
        if (dist < minDist)
            minDist = dist;
    }
    return minDist;
}

vector<double> ParticleFilter::getOffset(FieldMarker marker)
{
    double minDist = std::numeric_limits<double>::infinity();
    FieldMarker nearestTarget{};
    double dist;
    for (int i = 0; i < fieldMarkers.size(); i++)
    {
        auto target = fieldMarkers[i];
        if (target.type != marker.type)
        {
            continue;
        }
        dist = sqrt(pow((target.x - marker.x), 2.0) + pow((target.y - marker.y), 2.0));
        if (dist < minDist)
        {
            minDist = dist;
            nearestTarget = target;
        }
    }

    return vector<double>{nearestTarget.x - marker.x, nearestTarget.y - marker.y};
}

double ParticleFilter::residual(vector<FieldMarker> markers_r, Pose2D pose)
{
    double res = 0;

    for (int i = 0; i < markers_r.size(); i++)
    {
        auto marker_r = markers_r[i];
        double dist = max(norm(marker_r.x, marker_r.y), 0.1);
        auto marker_f = markerToFieldFrame(marker_r, pose);
        double conf = max(marker_r.confidence, 0.1);
        res += minDist(marker_f) * conf / 100.0 / dist * 3;
    }

    return res;
}

Pose2D ParticleFilter::finalAdjust(vector<FieldMarker> markers_r, Pose2D pose)
{
    if (markers_r.size() == 0)
        return Pose2D{0, 0, 0};

    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    for (int i = 0; i < markers_r.size(); i++)
    {
        auto marker_r = markers_r[i];
        auto marker_f = markerToFieldFrame(marker_r, pose);
        auto offset = getOffset(marker_f);
        dx += offset[0];
        dy += offset[1];
    }
    dx /= markers_r.size();
    dy /= markers_r.size();

    return Pose2D{pose.x + dx, pose.y + dy, pose.theta};
}

int ParticleFilter::calcProbs(vector<FieldMarker> markers_r)
{
    
    int rows = hypos.rows();
    if (rows < 1)
        return 1;

    for (int i = 0; i < rows; i++)
    {
        Pose2D pose{hypos(i, 0), hypos(i, 1), hypos(i, 2)};
        double res = residual(markers_r, pose);
        hypos(i, 3) = res;

        if (res < bestResidual)
        {
           
            bestResidual = res;
            bestPose = pose;
        }
    }

    double mean = hypos.col(3).mean();
    double sqSum = ((hypos.col(3) - mean).square().sum());
    double sigma = std::sqrt(sqSum / (rows - 1));
    double mu = hypos.col(3).minCoeff() - muOffset * sigma;

    for (int i = 0; i < rows; i++)
    {
        hypos(i, 4) = probDesity(hypos(i, 3), mu, sigma);
    }

    double probSum = hypos.col(4).sum();
    if (fabs(probSum) < 1e-5)
        return 1;

    hypos.col(4) = hypos.col(4) / probSum;

    double acc = 0;
    for (int i = 0; i < rows; i++)
    {
        acc += hypos(i, 4);
        hypos(i, 5) = acc;
    }

    return 0;
}

bool ParticleFilter::isConverged()
{
    return (
        (hypos.col(0).maxCoeff() - hypos.col(0).minCoeff() < convergeTolerance) && (hypos.col(1).maxCoeff() - hypos.col(1).minCoeff() < convergeTolerance) && (hypos.col(2).maxCoeff() - hypos.col(2).minCoeff() < convergeTolerance));
}

int ParticleFilter::locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraintsParam, Pose2D &pose, double &avgResidual, int numParticles, double offsetXParam, double offsetYParam, double offsetThetaParam)
{
    


    if (markers_r.size() < minMarkerCnt)
        return 4;

    bestResidual = std::numeric_limits<double>::infinity();
    bestPose = Pose2D{0.0, 0.0, 0.0};

    constraints = constraintsParam;
    offsetX = offsetXParam;
    offsetY = offsetYParam;
    offsetTheta = offsetThetaParam;

    genInitialParticles(numParticles);
    if (calcProbs(markers_r))
        return 5;

    for (int i = 0; i < maxIteration; i++)
    {
        if (isConverged())
        {

            avgResidual = bestResidual / markers_r.size();
            if (avgResidual > residualTolerance)
                return 2;

            pose = bestPose;
            pose.theta = toPInPI(pose.theta);
            return 0;
        }

        if (genParticles())
            return 1;

        if (calcProbs(markers_r))
            return 5;
    }

    return 3;
}

LocateResult ParticleFilter::locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraintsParam, int numParticles, double offsetXParam, double offsetYParam, double offsetThetaParam)
{
    auto start_time = chr::high_resolution_clock::now();
    LocateResult res;
    if (markers_r.size() < minMarkerCnt) // 如果标记点数量小于最小标记点数量，返回错误
    {
        res.success = false;
        res.code = 4;
        res.msecs = msecsSince(start_time);
        return res;
    }

    bestResidual = std::numeric_limits<double>::infinity();
    bestPose = Pose2D{0.0, 0.0, 0.0};

    constraints = constraintsParam;
    offsetX = offsetXParam;
    offsetY = offsetYParam;
    offsetTheta = offsetThetaParam;

    genInitialParticles(numParticles);

    if (calcProbs(markers_r))
    {
        res.success = false;
        res.code = 5;
        res.msecs = msecsSince(start_time);
        return res;
    }

    for (int i = 0; i < maxIteration; i++)
    {
        if (isConverged())
        {
            std::cout << "bestResidual: " << bestResidual<<endl;
            res.residual = bestResidual / markers_r.size();
            std::cout << "res.residual: " << res.residual<<endl;
            if (res.residual > residualTolerance)
            {       
                res.success = false;
                res.code = 2;
                res.msecs = msecsSince(start_time);
                return res;
            }

            res.success = true;
            res.code = 0;
            res.pose = bestPose;
            res.pose.theta = toPInPI(res.pose.theta);
            res.msecs = msecsSince(start_time);
            return res;
        }

        if (genParticles())
        {
            res.success = false;
            res.code = 1;
            res.msecs = msecsSince(start_time);
            return res;
        }

        if (calcProbs(markers_r))
        {
            res.success = false;
            res.code = 5;
            res.msecs = msecsSince(start_time);
            return res;
        }
    }

    res.success = false;
    res.code = 3; 
    res.msecs = msecsSince(start_time);
    return res;
}