#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"

#include "publish_selected_patch.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QVariant>

#include <algorithm>    
using namespace std;

namespace publish_selected_patch
{
PublishSelectedPatch::PublishSelectedPatch()
{
  updateTopic();
}

PublishSelectedPatch::~PublishSelectedPatch()
{
}

void PublishSelectedPatch::updateTopic()
{
  // nh_.param("frame_id", tf_frame_, std::string("/base_link"));
  cloud_topic_ = "/selected_patch";
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>( cloud_topic_.c_str(), 1 );
  // ROS_INFO( "Publishing data on topic %s with frame_id %s.",
  //           nh_.resolveName (cloud_topic_).c_str (),
  //           tf_frame_.c_str() );
}

int PublishSelectedPatch::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  int flags = rviz::SelectionTool::processMouseEvent( event );

  // determine current selection mode
  if( event.alt() )
  {
    selecting_ = false;
  }
  else
  {
    if( event.leftDown() )
    {
      selecting_ = true;
      sel_start_x_ = event.x;
      sel_start_y_ = event.y;
    }
  }
  
  // We use get3DPoint( event.viewport, temp_x, temp_y, pos ) to replace the getSelection()
  // and pcl::PointCloud to construct the point cloud.
  if( selecting_ )
  {
    if( event.leftUp() )
    {
      vector<Ogre::Vector3> result_points;
      int x_left = min(sel_start_x_, event.x);
      int x_right = max(sel_start_x_, event.x);
      int y_left = min(sel_start_y_, event.y);
      int y_right = max(sel_start_y_, event.y);
      for(int temp_x = x_left; temp_x <= x_right; temp_x++) {
        for(int temp_y = y_left; temp_y <= y_right; temp_y++) {
          Ogre::Vector3 pos;
          bool success = context_->getSelectionManager()->get3DPoint( event.viewport, temp_x, temp_y, pos );
          if ( success ) result_points.push_back(pos);
        }
      }

      int num_points = result_points.size();     
      if( num_points <= 0 )
      {
        return flags;
      }

      sensor_msgs::PointCloud2 selected_cloud;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl_clr_ptr->points.resize(num_points);
      pcl_clr_ptr->width = num_points;
      pcl_clr_ptr->height = 1;
      pcl_clr_ptr->header.frame_id = context_->getFixedFrame().toStdString();

      pcl::PointXYZRGB the_clr_point;
      for (int ipt = 0; ipt < num_points; ipt++) {
          the_clr_point.x = result_points[ipt].x;
          the_clr_point.y = result_points[ipt].y;
          the_clr_point.z = result_points[ipt].z;
          the_clr_point.r = 0;
          the_clr_point.g = 0;
          the_clr_point.b = 0;
          pcl_clr_ptr->points[ipt] = the_clr_point;
      }

      pcl::toROSMsg(*pcl_clr_ptr, selected_cloud);

      selected_cloud.header.stamp = ros::Time::now();
      pub_.publish(selected_cloud);
    }
  }

  return flags;
}

} // end namespace publish_selected_patch

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( publish_selected_patch::PublishSelectedPatch, rviz::Tool )
