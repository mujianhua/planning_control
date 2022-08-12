#pragma once

#include <mutex>
#include <thread>

#include <planning/CenterLine.h>
#include <ros/ros.h>

#include "data_struct.h"
#include "planning/reference_line.h"

namespace planning {

// TODO: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// TODO: how to get reference points...

class ReferenceLineProvider {
 public:
  ReferenceLineProvider();

  void ReferenceLineSubCb(const CenterLineConstPtr &msg);

  bool Start();

  void Stop();

  bool GetReferenceLine(ReferenceLine *reference_line);

 private:
  void GenerateThread();

  bool CreateReferenceLine(ReferenceLine &reference_line);

  bool UpdateReferenceLine(const ReferenceLine &reference_line);

 private:
  ros::NodeHandle n;
  ros::Subscriber reference_points_sub_;
  CenterLineConstPtr msg_;
  std::vector<TrajectoryPoint> raw_reference_points_;
  ReferenceLine reference_line_;

  std::atomic<bool> is_stop_{false};
  std::thread thread_;
  std::mutex mutex_;
};

}  // namespace planning
