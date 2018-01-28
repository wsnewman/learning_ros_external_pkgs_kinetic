################################ Overview ##################################

The tool, “publish selected points”, is a useful tool for immediately grasping a group of points in rviz. However, we find it publish the points with unmatched frame id especially when we are using Kinect.
################################ Bug description ##################################

The rviz plugin “Slected Points Publisher” (earlier version: “Publish Selected Points”) is a very handy tool that user can drag with the left button to select objects in the 3D scene, and the coordinates of points within selected area would be published. However, we found certain issue exists when applying the tool.

When we use camera/kinect (PointCloud2 topic “/camera/depth_registered/points” and “/kinect/depth/points” respectively) to view object, the coordinates of the selected points using either version of the tool would always be with respect to the same frame no matter what fixed frame we select in rviz. Check the test video below:

https://www.youtube.com/watch?v=bWeUrPW1KcI

https://www.youtube.com/watch?v=IBSn1IL3aYE

In the first video, we select different points on the shelf to check if the plugin really takes the fixed frame (Axes shown in rviz view) as its frame. After testing several points we can see that the fixed frame in rviz was selected as base_link, a frame with x-axis pointing forward, z-axis pointing upward. But the published coordinates of selected points tell us that it takes a different frame as fixed frame instead of base_link, a frame with z-axis pointing forward, x-axis pointing to the right. 

In the second video, we compare two cases when the fixed frame we set is base_link and odom respectively. The mobot has been moved forward manually to make sure certain difference should exist using this two fixed frame if the tool work properly. However, through testing we found that with either frame set as fixed frame, there are no difference on the coordinates of selected points. Still, both of them take that weird frame (with z-axis pointing forward, x-axis pointing to the right) as the fixed frame. 

This bug about frame id really makes the tool inconvenient to use, and it exists when using rviz default tool “Select” as well. But before we fix the bug, we found some interesting phenomena. When we use laser scan to view scene in rviz (with LaserScan topic “/scan”), there is no such problem using the tool. Check the following video:

https://www.youtube.com/watch?v=dUhovFmPqzQ

In this video we can see that we set the fixed frame as base_link first, and by selecting different points and comparing their coordinates we can tell the tool does take the base_link as its fixed frame. Then we change the fixed frame field into odom(again, the mobot has been moved forward manually from its origin), and difference on x coordinates does exist when we select points on the back of shelf with different fixed frame. Thus we can say that when using laser scan instead of camera, this tool works very well with no frame bug.  

################################ Fix in publish_selected_points plugin ##################################

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

We use get3DPoint( event.viewport, temp_x, temp_y, pos ) to replace the getSelection() and pcl::PointCloud to construct the point cloud.

################################### Trying It Out ####################################

Once your RViz plugin is compiled and exported, simply run rviz normally:

$ rosrun rviz rviz 

and rviz will use pluginlib to find all the plugins exported to it.

Add a PublishSelectedPatch tool by clicking on the “+” button in the toolbar and selecting “PublishSelectedPatch” from the list under your plugin package name (here it is “publish_selected_patch”).

Now you can check the point cloud information by 

$ rostopic echo /selected_patch

################################################################################

