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
#include <multimap_server_msgs/SaveMapColour.h>

using namespace std;

class MapSaver {
public:
  MapSaver() {
    save_map_service =
        n.advertiseService("save_map", &MapSaver::saveMapCallback, this);
    save_map_colou_service = 
        n.advertiseService("save_map_colour", &MapSaver::saveMapColourCallback, this);
  }

  ros::NodeHandle n;
  ros::ServiceServer save_map_service;
  ros::ServiceServer save_map_colou_service;
  ros::ServiceClient get_map_client;

  
  /**
   * @brief Splits a string into a vector of substrings based on a delimiter character.
   *
   * This function takes a string `s` and splits it into substrings wherever the 
   * delimiter character `c` is found. The resulting substrings are stored in the 
   * provided vector `v`.
   *
   * @param s The input string to be split.
   * @param c The delimiter character used to split the string.
   * @param v The vector where the resulting substrings will be stored.
   */
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

  /**
   * @brief Saves the occupancy grid map to a PGM file and its metadata to a YAML file.
   * 
   * This function writes the occupancy grid map data to a PGM file and the corresponding
   * metadata to a YAML file. The map is saved with specified color values for occupied,
   * free, and unknown cells.
   * 
   * @param filename The base name of the files to save (without extension).
   * @param map The occupancy grid map to save.
   * @param threshold_occupied The threshold above which cells are considered occupied.
   * @param threshold_free The threshold below which cells are considered free.
   * @param colour_occupied The color value for occupied cells in the PGM file (default is 0).
   * @param colour_free The color value for free cells in the PGM file (default is 254).
   * @param colour_unknown The color value for unknown cells in the PGM file (default is 205).
   * @return true if the map and metadata were successfully saved, false otherwise.
   */
  bool saveMap(const std::string &filename, const nav_msgs::OccupancyGrid &map,
               const int &threshold_occupied, const int &threshold_free, 
               const int16_t &colour_occupied = 0, const int16_t &colour_free = 254, 
               const int16_t &colour_unknown = 205) {

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
          fputc(colour_free, out);
        } else if (map.data[i] <= 100 &&
                   map.data[i] >= threshold_occupied) { // occ (0.65,1]
          fputc(colour_occupied, out);
        } else { // occ [0.1,0.65]
          fputc(colour_unknown, out);
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

  /**
   * @brief Callback function to save a map.
   *
   * This function handles the request to save a map by either calling a map service
   * or waiting for a map topic. It validates the thresholds for occupied and free
   * spaces, retrieves the map, and saves it to the specified file.
   *
   * @param req The request containing the map service name, filename, and thresholds.
   * @param res The response indicating success or failure and a message.
   * @return true Always returns true to indicate the service call was processed.
   */
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


  /**
   * @brief Callback function to save a map with specified color and threshold settings.
   * 
   * This function handles the request to save a map with specified color and threshold settings.
   * It retrieves the map from a specified service or topic, applies the given thresholds and colors,
   * and saves the map to a file.
   * 
   * @param req The request containing the parameters for saving the map.
   * @param res The response indicating the success or failure of the operation.
   * @return true Always returns true to indicate that the service call was handled.
   * 
   * Request Parameters:
   * - use_default_thresholds: Boolean flag to indicate whether to use default thresholds.
   * - threshold_occupied: Occupied threshold value (1-100).
   * - threshold_free: Free threshold value (0-100).
   * - colour_occupied: Color value for occupied cells (0-254).
   * - colour_free: Color value for free cells (0-254).
   * - colour_unknown: Color value for unknown cells (0-254).
   * - map_service: The name of the map service or topic to retrieve the map from.
   * - map_filename: The filename to save the map to.
   * 
   * Response Parameters:
   * - success: Boolean flag indicating whether the map was saved successfully.
   * - msg: Message providing additional information about the result.
   */
  bool saveMapColourCallback(multimap_server_msgs::SaveMapColour::Request &req,
                       multimap_server_msgs::SaveMapColour::Response &res) {
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

      if (req.colour_occupied < 0 || req.colour_occupied > 254) {
        res.success = false;
        res.msg = "colour_occupied must be between 0 and 254";
        return true;
      }
      if (req.colour_free < 0 || req.colour_free > 254) {
        res.success = false;
        res.msg = "colour_free must be between 0 and 254";
        return true;
      }
      if (req.colour_unknown < 0 || req.colour_unknown > 254) {
        res.success = false;
        res.msg = "colour_unknown must be between 0 and 254";
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

        if( req.use_default_thresholds == false ) {
          if (saveMap(req.map_filename, map, threshold_occupied,
                      threshold_free, req.colour_occupied, req.colour_free, req.colour_unknown) == true) {
            res.success = true;
            res.msg = "Map saved succesfully";
            return true;
          } else {
            res.success = false;
            res.msg = "Error saving the map";
            return true;
          }
        } else {
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
        }
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
