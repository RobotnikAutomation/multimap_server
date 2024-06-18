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

#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/GetMap.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cstdio>
#include <multimap_server_msgs/SaveMap.h>

using namespace std;

class MapSaver {
public:
  MapSaver() {
    save_map_service =
        n.advertiseService("save_map", &MapSaver::saveMapCallback, this);
  }

  ros::NodeHandle n;
  ros::ServiceServer save_map_service;
  ros::ServiceClient get_map_client;

  // splits a string 's' by the delimeter 'c' and returns the result into 'v'
  void split(const string &s, char c, vector<string> &v) {
    string::size_type i = 0;
    string::size_type j = s.find(c);

    while (j != string::npos) {
      v.push_back(s.substr(i, j - i));
      i = ++j;
      j = s.find(c, j);

      if (j == string::npos)
        v.push_back(s.substr(i, s.length()));
    }
  }

  /// @brief the map to a pgm file and a yaml file
  /// @param mapname name of the map
  /// @param map the map to save
  /// @param threshold_occupied cells with occupancy probability greater than
  /// this value are considered occupied
  /// @param threshold_free cells with occupancy probability less than this
  /// value are considered free
  /// @return true if the map is saved, false if map could not be saved
  bool saveMap(const std::string &filename, const nav_msgs::OccupancyGrid &map,
               const int &threshold_occupied, const int &threshold_free) {

    std::string mapdatafile = filename + ".pgm";
    ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE *out = fopen(mapdatafile.c_str(), "w");
    if (!out) {
      ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
      return false;
    }

    fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
            map.info.resolution, map.info.width, map.info.height);
    for (unsigned int y = 0; y < map.info.height; y++) {
      for (unsigned int x = 0; x < map.info.width; x++) {
        unsigned int i = x + (map.info.height - y - 1) * map.info.width;
        if (map.data[i] >= 0 && map.data[i] <= threshold_free) { // occ [0,0.1)
          fputc(254, out);
        } else if (map.data[i] <= 100 &&
                   map.data[i] >= threshold_occupied) { // occ (0.65,1]
          fputc(000, out);
        } else { // occ [0.1,0.65]
          fputc(205, out);
        }
      }
    }

    fclose(out);

    std::string mapmetadatafile = filename + ".yaml";
    std::string pgm_filename = mapdatafile;

    // extracts just the filename to be saved inside the yaml file
    std::vector<std::string> result;
    split(mapdatafile, '/', result);
    if (result.size() > 0) {
      // for (size_t i = 0; i < result.size(); i++)
      //  ROS_INFO("result %d:  %s", (int)i, result[i].c_str());
      pgm_filename = result.back();
    }

    ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
    FILE *yaml = fopen(mapmetadatafile.c_str(), "w");

    geometry_msgs::Quaternion orientation = map.info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y,
                                       orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);
    if (std::isnan(yaw)) {
      yaw = 0.0;
    }
    fprintf(yaml,
            "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: "
            "0\noccupied_thresh: 0.65\nfree_thresh: "
            "0.196\n\n",
            pgm_filename.c_str(), map.info.resolution,
            map.info.origin.position.x, map.info.origin.position.y, yaw);

    fclose(yaml);

    return true;
  }

  // TODO: Saved in specified directory
  bool saveMapCallback(multimap_server_msgs::SaveMap::Request &req,
                       multimap_server_msgs::SaveMap::Response &res) {
    std::string mapname = "map";
    int threshold_occupied = 100;
    int threshold_free = 0;

    if (req.use_default_thresholds == false) {
      threshold_occupied = req.threshold_occupied;
      threshold_free = req.threshold_free;

      if (threshold_occupied < 1 || threshold_occupied > 100) {
        res.success = false;
        res.msg = "threshold_occupied must be between 1 and 100";
        return true;
      }
      if (threshold_free < 0 || threshold_free > 100) {
        res.success = false;
        res.msg = "threshold_free must be between 0 and 100";
        return true;
      }
    }

    get_map_client = n.serviceClient<nav_msgs::GetMap>(req.map_service.c_str());
    nav_msgs::GetMap getMap;

    if (get_map_client.exists()) {
      if (get_map_client.call(getMap)) {
        ROS_INFO("Received a %d X %d map @ %.3f m/pix",
                 getMap.response.map.info.width,
                 getMap.response.map.info.height,
                 getMap.response.map.info.resolution);

        if (saveMap(req.map_filename, getMap.response.map, threshold_occupied,
                    threshold_free) == true) {
          res.success = true;
          res.msg = "Map saved succesfully";
          return true;
        } else {
          res.success = false;
          res.msg = "Error saving the map";
          return true;
        }
      } else {
        res.success = false;
        res.msg = "Map couldn't be retrieved. Service " + req.map_service +
                  " returned an error";
        return true;
      }
    } else {
      ROS_WARN_STREAM("Service " << req.map_service << " does not exist");

      // Check if the topic exists to retrieve the map
      boost::shared_ptr<nav_msgs::OccupancyGrid const> ret_map;
      nav_msgs::OccupancyGrid map;

      ret_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(
          req.map_service, ros::Duration(5));
      if (ret_map != NULL) {
        // Topic exists
        map = *ret_map;
        ROS_INFO("Received a %d X %d map @ %.3f m/pix", map.info.width,
                 map.info.height, map.info.resolution);

        if (saveMap(req.map_filename, map, threshold_occupied,
                    threshold_free) == true) {
          res.success = true;
          res.msg = "Map saved succesfully";
          return true;
        } else {
          res.success = false;
          res.msg = "Error saving the map";
          return true;
        }
      } else {
        res.success = false;
        res.msg = "No Service nor Topic with namespace " + req.map_service +
                  " does exist";
        ROS_ERROR_STREAM(res.msg);
        return true;
      }
      return true;
    }

    return true;
  }
};

#define USAGE                                                                  \
  "Usage: \n"                                                                  \
  "  map_saver\n"

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_saver");

  MapSaver map_saver;

  ros::spin();
  return 0;
}
