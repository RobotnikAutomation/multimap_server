/*
 * map_saver
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <multimap_server_msgs/SaveMap.h>

using namespace std;

class MapSaver
{
public:
  MapSaver()
  {
    save_map_service = n.advertiseService("save_map", &MapSaver::saveMapCallback, this);
  }

  ros::NodeHandle n;
  ros::ServiceServer save_map_service;
  ros::ServiceClient get_map_client;

  // TODO: Saved in specified directory
  bool saveMapCallback(multimap_server_msgs::SaveMap::Request& req, multimap_server_msgs::SaveMap::Response& res)
  {
    std::string mapname = "map";
    int threshold_occupied = 100;
    int threshold_free = 0;

    if (req.use_default_thresholds == false)
    {
      threshold_occupied = req.threshold_occupied;
      threshold_free = req.threshold_free;

      if (threshold_occupied < 1 || threshold_occupied > 100)
      {
        res.success = false;
        res.msg = "threshold_occupied must be between 1 and 100";
        return true;
      }
      if (threshold_free < 0 || threshold_free > 100)
      {
        res.success = false;
        res.msg = "threshold_free must be between 0 and 100";
        return true;
      }
    }

    get_map_client = n.serviceClient<nav_msgs::GetMap>(req.map_service.c_str());
    nav_msgs::GetMap getMap;

    if (get_map_client.exists())
    {
      if (get_map_client.call(getMap))
      {
        ROS_INFO("Received a %d X %d map @ %.3f m/pix", getMap.response.map.info.width, getMap.response.map.info.height,
                 getMap.response.map.info.resolution);

        std::string mapdatafile = req.map_filename + ".pgm";
        ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
        FILE* out = fopen(mapdatafile.c_str(), "w");
        if (!out)
        {
          ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
          res.success = false;
          res.msg = "Couldn't save map file to " + mapdatafile;
          return true;
        }

        fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n", getMap.response.map.info.resolution,
                getMap.response.map.info.width, getMap.response.map.info.height);
        for (unsigned int y = 0; y < getMap.response.map.info.height; y++)
        {
          for (unsigned int x = 0; x < getMap.response.map.info.width; x++)
          {
            unsigned int i = x + (getMap.response.map.info.height - y - 1) * getMap.response.map.info.width;
            if (getMap.response.map.data[i] >= 0 && getMap.response.map.data[i] <= threshold_free)
            {  // occ [0,0.1)
              fputc(254, out);
            }
            else if (getMap.response.map.data[i] <= 100 && getMap.response.map.data[i] >= threshold_occupied)
            {  // occ (0.65,1]
              fputc(000, out);
            }
            else
            {  // occ [0.1,0.65]
              fputc(205, out);
            }
          }
        }

        fclose(out);

        std::string mapmetadatafile = req.map_filename + ".yaml";
        ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
        FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

        geometry_msgs::Quaternion orientation = getMap.response.map.info.origin.orientation;
        tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);

        fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: "
                      "0.196\n\n",
                mapdatafile.c_str(), getMap.response.map.info.resolution, getMap.response.map.info.origin.position.x,
                getMap.response.map.info.origin.position.y, yaw);

        fclose(yaml);

        ROS_INFO("Done\n");
        res.success = true;
        res.msg = "Map saved succesfully";
        return true;
      }
      else
      {
        res.success = false;
        res.msg = "Map couldn't be retrieved. Service " + req.map_service + " returned an error";
        return true;
      }
    }
    else
    {
      res.success = false;
      res.msg = "Service " + req.map_service + " does not exist";
      return true;
    }

    return true;
  }
};

#define USAGE                                                                                                          \
  "Usage: \n"                                                                                                          \
  "  map_saver\n"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");

  MapSaver map_saver;

  ros::spin();
  return 0;
}
