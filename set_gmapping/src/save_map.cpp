#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

using std::string;

class MapGenerator
{
  /*
   *Copyright (c) 2008, Willow Garage, Inc.
   *All rights reserved.
   */
  public:
    MapGenerator(const string& mapname, const string& file_location, int threshold_occupied, int threshold_free)
      : mapname_(mapname), file_location_(file_location), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
      ros::NodeHandle n;
      ROS_INFO("save_map:Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ROS_INFO("save_map:Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);

      string mapdatafile = mapname_ + ".pgm";
      string mapdata_location_name = file_location_ + mapname_ + ".pgm";
      ROS_INFO("save_map:Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdata_location_name.c_str(), "w");	//只写方式打开文件
      if (!out)
      {
        ROS_ERROR("save_map:Couldn't save map file to %s", mapdatafile.c_str());
        return;
      }

      fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;
          if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
            fputc(254, out);
          } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
            fputc(000, out);
          } else { //occ [0.25,0.65]
            fputc(205, out);
          }
        }
      }

      fclose(out);

      string mapmetadatafile = mapname_ + ".yaml";
      string mapmeta_location_name = file_location_ + mapname_ + ".yaml";
      ROS_INFO("save_map:Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmeta_location_name.c_str(), "w");


      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
       */

      geometry_msgs::Quaternion orientation = map->info.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      ));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

      fclose(yaml);

      ROS_INFO("save_map:Done\n");
      saved_map_ = true;
    }

    string mapname_;
    string file_location_;
    ros::Subscriber map_sub_;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;

};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "save_map");
    ros::NodeHandle nh_sm;
    
    string mapname,file_location;
    int threshold_occupied,threshold_free,rate;

    nh_sm.param("/save_map/rate",rate,65);
    nh_sm.param("/save_map/threshold_occupied",threshold_occupied,65);
    nh_sm.param("/save_map/threshold_free",threshold_free,25);
    nh_sm.param<string>("/save_map/mapname",mapname,"map");
    nh_sm.param<string>("/save_map/file_location",file_location,"~/Documents/");

    if (threshold_occupied <= threshold_free){
        ROS_ERROR("threshold_free must be smaller than threshold_occupied");
        return 1;
    }

    ros::Rate loop_rate(rate);
    while(ros::ok()){        
        MapGenerator mg(mapname, file_location, threshold_occupied, threshold_free);
        while(!mg.saved_map_ && ros::ok())
            ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
