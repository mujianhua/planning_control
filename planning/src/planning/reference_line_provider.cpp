#include "planning/reference_line_provider.h"

#include "planning/planning_config.h"
#include "planning/reference_line.h"

namespace planning {

ReferenceLineProvider::ReferenceLineProvider() {
  reference_points_sub_ = n.subscribe(
      "/center_line", 1, &ReferenceLineProvider::ReferenceLineSubCb, this);
}

ReferenceLineProvider::~ReferenceLineProvider() { Stop(); }

void ReferenceLineProvider::ReferenceLineSubCb(const CenterLineConstPtr &msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto &pt : msg->points) {
    TrajectoryPoint tp;
    tp.s = pt.s;
    tp.x = pt.x;
    tp.y = pt.y;
    tp.theta = pt.theta;
    tp.kappa = pt.kappa;
    tp.left_bound = pt.left_bound;
    tp.right_bound = pt.right_bound;
    raw_reference_points_.push_back(tp);
  }
}

bool ReferenceLineProvider::Start() {
  if (FLAGS_enable_reference_line_provider_thread) {
    thread_ = std::thread(&ReferenceLineProvider::GenerateThread, this);
  }
  return true;
}

void ReferenceLineProvider::Stop() { is_stop_ = true; }

bool ReferenceLineProvider::GetReferenceLine(ReferenceLine *reference_line) {
  if (FLAGS_enable_reference_line_provider_thread) {
    std::lock_guard<std::mutex> lock(mutex_);
    *reference_line = reference_line_;
  } else {
    CreateReferenceLine(reference_line);
    UpdateReferenceLine(*reference_line);
  }
  return true;
}

void ReferenceLineProvider::GenerateThread() {
  while (!is_stop_) {
    static constexpr double kSleepTime = 0.5;
    ros::Duration(kSleepTime).sleep();
    ros::Time start_time = ros::Time::now();
    ROS_DEBUG_NAMED("Reference Line Provider", "reference line updating...");
    ReferenceLine reference_line;
    CreateReferenceLine(&reference_line);
    UpdateReferenceLine(reference_line);
  }
  ROS_DEBUG("[Reference Line Provider] thread end.");
}

bool ReferenceLineProvider::CreateReferenceLine(ReferenceLine *reference_line) {
  std::vector<TrajectoryPoint> raw_reference_points;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    raw_reference_points = raw_reference_points_;
  }

  for (size_t i = 1; i < raw_reference_points.size(); i++) {
    raw_reference_points[i].dkappa =
        (raw_reference_points[i].kappa - raw_reference_points[i - 1].kappa) /
        (raw_reference_points[i].s - raw_reference_points[i - 1].s);
  }
  *reference_line = ReferenceLine(raw_reference_points);
  return true;
}

bool ReferenceLineProvider::UpdateReferenceLine(
    const planning::ReferenceLine &reference_line) {
  std::lock_guard<std::mutex> lock(mutex_);
  reference_line_ = reference_line;
  return true;
}

}  // namespace planning
