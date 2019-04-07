#ifndef ORCA_MISSION_H
#define ORCA_MISSION_H

#include "nav_msgs/msg/path.hpp"

#include "orca_base/orca_motion.hpp"

namespace orca_base {

//=====================================================================================
// BaseMission is an abstract base class for running missions
//=====================================================================================

class BaseMission
{
protected:

  rclcpp::Logger logger_;
  std::unique_ptr<BaseMotion> planner_;
  //OrcaPose pose_;

  rclcpp::Time last_time_;  // Time of last call to advance
  double dt_;               // Elapsed time since the last call to advance (s)

public:

  BaseMission(rclcpp::Logger logger): logger_{logger} {}

  virtual bool init(const rclcpp::Time now, const OrcaPose &goal, OrcaOdometry &plan) = 0;
  virtual bool advance(const rclcpp::Time now, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar);

  static void add_to_path(nav_msgs::msg::Path &path, const OrcaPose &pose);
  static void add_to_path(nav_msgs::msg::Path &path, const std::vector<OrcaPose> &poses);
};

//=====================================================================================
// SurfaceMission
//
// Run from pose A to B
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

class SurfaceMission: public BaseMission
{
private:

  enum class Phase
  {
    no_goal,    // There's no goal
    turn,       // We're turning toward the goal
    run,        // We're moving toward the goal
    final_turn  // We're turning to match the goal heading
  };

  Phase phase_ = Phase::no_goal;

  OrcaPose goal1_;
  OrcaPose goal2_;
  OrcaPose goal3_;

public:

  SurfaceMission(rclcpp::Logger logger): BaseMission(logger) {}

  bool init(const rclcpp::Time now, const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(const rclcpp::Time now, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar) override;
};

//=====================================================================================
// SquareMission
//
// Run in a square defined by 2 poses
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

class SquareMission: public BaseMission
{
private:

  enum class Planner
  {
    vertical,
    rotate,
    line
  };

  struct Segment
  {
    Planner planner;
    OrcaPose goal;
  };

  int segment_;
  std::vector<Segment>segments_;

public:

  SquareMission(rclcpp::Logger logger): BaseMission(logger) {}

  bool init(const rclcpp::Time now, const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(const rclcpp::Time now, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar) override;
};

//=====================================================================================
// ArcMission
//
// Run in an arc defined by 2 poses
// Orient the vehicle in the direction of motion to avoid obstacles
//=====================================================================================

class ArcMission: public BaseMission
{
public:

  ArcMission(rclcpp::Logger logger): BaseMission(logger) {}

  bool init(const rclcpp::Time now, const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(const rclcpp::Time now, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar) override;
};

//=====================================================================================
// VerticalMission
//
// Descend or ascend
//=====================================================================================

class VerticalMission: public BaseMission
{
public:

  VerticalMission(rclcpp::Logger logger): BaseMission(logger) {}

  bool init(const rclcpp::Time now, const OrcaPose &goal, OrcaOdometry &plan) override;
  bool advance(const rclcpp::Time now, const OrcaPose &curr, OrcaOdometry &plan, OrcaPose &u_bar) override;
};

} // namespace orca_base

#endif // ORCA_MISSION_H