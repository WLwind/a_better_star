#include <a_better_star/orientation_filter.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>

namespace a_better_star
{

void set_angle(geometry_msgs::PoseStamped* pose, double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  tf2::convert(q, pose->pose.orientation);
}

void OrientationFilter::processPath(const geometry_msgs::PoseStamped& start, 
                                    std::vector<geometry_msgs::PoseStamped>& path)
{
    int n = path.size();
    if (n == 0) return;
    switch(omode_) {
        case FORWARD:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
            }
            break;
        case BACKWARD:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angles::normalize_angle(tf2::getYaw(path[i].pose.orientation) + M_PI));
            }
            break;
        case LEFTWARD:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angles::normalize_angle(tf2::getYaw(path[i].pose.orientation) - M_PI_2));
            }
            break;
        case RIGHTWARD:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], angles::normalize_angle(tf2::getYaw(path[i].pose.orientation) + M_PI_2));
            }
            break;
        case INTERPOLATE:
            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, 0, n-1);
            break;
        case FORWARDTHENINTERPOLATE:
            for(int i=0;i<n-1;i++){
                setAngleBasedOnPositionDerivative(path, i);
            }
            
            int i=n-3;
            const double last = tf2::getYaw(path[i].pose.orientation);
            while( i>0 ){
                const double new_angle = tf2::getYaw(path[i-1].pose.orientation);
                double diff = fabs(angles::shortest_angular_distance(new_angle, last));
                if( diff>0.35)
                    break;
                else
                    i--;
            }
            
            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, i, n-1);
            break;           
    }
}
    
void OrientationFilter::setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path, int index)
{
  int index0 = std::max(0, index - window_size_);
  int index1 = std::min((int)path.size() - 1, index + window_size_);

  double x0 = path[index0].pose.position.x,
         y0 = path[index0].pose.position.y,
         x1 = path[index1].pose.position.x,
         y1 = path[index1].pose.position.y;
         
  double angle = atan2(y1-y0,x1-x0);
  set_angle(&path[index], angle);
}

void OrientationFilter::interpolate(std::vector<geometry_msgs::PoseStamped>& path, 
                                    int start_index, int end_index)
{
    const double start_yaw = tf2::getYaw(path[start_index].pose.orientation),
                 end_yaw   = tf2::getYaw(path[end_index  ].pose.orientation);
    double diff = angles::shortest_angular_distance(start_yaw, end_yaw);
    double increment = diff/(end_index-start_index);
    for(int i=start_index; i<=end_index; i++){
        double angle = start_yaw + increment * i;
        set_angle(&path[i], angle);
    }
}
}
