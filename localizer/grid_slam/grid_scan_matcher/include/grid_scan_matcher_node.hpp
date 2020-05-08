#ifndef GRIDSCANMATCHERNODE
#define GRIDSCANMATCHERNODE

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <localization_msgs/LPoint2D.h>
#include <localization_msgs/LPoint2DArray.h>
#include <localization_msgs/MatchingResult.h>
#include <LPoint2DArrayClass.hpp>
#include <ProbabilityGridMap.hpp>
#include <MultiResolutionGridMap.hpp>
#include <CeresGridPoseOptimizer.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include <cmath>
#include <vector>
#include <std_msgs/Bool.h>
#include <chrono>
#include <Utils.hpp>

#endif
