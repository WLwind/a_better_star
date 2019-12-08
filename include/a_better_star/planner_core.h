#pragma once

#define POT_HIGH 1.0e10        // unassigned cell potential
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <a_better_star/potential_calculator.h>
#include <a_better_star/expander.h>
#include <a_better_star/traceback.h>
#include <a_better_star/orientation_filter.h>
#include <a_better_star/GlobalPlannerConfig.h>

namespace a_better_star
{

class Expander;//Forward declaration
class GridPath;

/**
 * @class GlobalPlanner
 * @brief Provides a ROS wrapper for the a_better_star planner which is a modification of official global_planner.
 */

class GlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
    /**
    * @brief  Default constructor
    */
    GlobalPlanner();

    /**
    * @brief  Constructor
    * @param  name The name of this planner
    * @param  costmap A pointer to the costmap to use
    * @param  frame_id Frame of the costmap
    */
    GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

    /**
    * @brief  Default destructor for the PlannerCore object
    */
    ~GlobalPlanner();

    /**
    * @brief  Initialization function for the PlannerCore object
    * @param  name The name of this planner
    * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
    */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

    /**
    * @brief Given a goal pose in the world, compute a plan
    * @param start The start pose
    * @param goal The goal pose
    * @param plan The plan... filled by the planner
    * @return True if a valid plan was found, false otherwise
    */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan) override;

    /**
    * @brief Given a goal pose in the world, compute a plan
    * @param start The start pose
    * @param goal The goal pose
    * @param tolerance The tolerance on the goal point for the planner
    * @param plan The plan... filled by the planner
    * @return True if a valid plan was found, false otherwise
    */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                    std::vector<geometry_msgs::PoseStamped>& plan);

    /**
    * @brief  Computes the full navigation function for the map given a point in the world to start from
    * @param world_point The point to use for seeding the navigation function
    * @return True if the navigation function was computed successfully, false otherwise
    */
    bool computePotential(const geometry_msgs::Point& world_point);

    /**
    * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
    * @param start_x
    * @param start_y
    * @param end_x
    * @param end_y
    * @param goal The goal pose to create a plan to
    * @param plan The plan... filled by the planner
    * @return True if a valid plan was found, false otherwise
    */
    bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan);

    /**
    * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
    * @param world_point The point to get the potential for
    * @return The navigation function's value at that point in the world
    */
    double getPointPotential(const geometry_msgs::Point& world_point);

    /**
    * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
    * @param world_point The point to get the potential for
    * @return True if the navigation function is valid at that point in the world, false otherwise
    */
    bool validPointPotential(const geometry_msgs::Point& world_point);

    /**
    * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
    * @param world_point The point to get the potential for
    * @param tolerance The tolerance on searching around the world_point specified
    * @return True if the navigation function is valid at that point in the world, false otherwise
    */
    bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

    /**
    * @brief  Publish a path for visualization purposes
    */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    /**
    * @brief  A ROS service server for making a new global plan
    */
    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

protected:

    /**
    * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
    */
    costmap_2d::Costmap2D* costmap_;
    std::string frame_id_;
    ros::Publisher plan_pub_;
    bool initialized_, allow_unknown_;

private:
    void mapToWorld(double mx, double my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, double& mx, double& my);
    void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
    void publishPotential(float* potential);

    double planner_window_x_, planner_window_y_, default_tolerance_;
    boost::mutex mutex_;
    ros::ServiceServer make_plan_srv_;

    PotentialCalculator* p_calc_;//calculate g(n)
    Expander* planner_;//find path
    Traceback* path_maker_;//trace back
    OrientationFilter* orientation_filter_;//set orientation of each point

    bool publish_potential_;
    ros::Publisher potential_pub_;
    int publish_scale_;

    void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
    unsigned char* cost_array_;
    float* potential_array_;//g(n)
    unsigned int start_x_, start_y_, end_x_, end_y_;

    bool old_navfn_behavior_;
    float convert_offset_;

    dynamic_reconfigure::Server<GlobalPlannerConfig> *dsrv_;//dynamic configuration server
    /**
    * @brief Dynamic configuration callback function
    */
    void reconfigureCB(GlobalPlannerConfig &config, uint32_t level);

};

} //end namespace
