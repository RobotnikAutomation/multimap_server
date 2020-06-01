/*
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/* Author: Brian Gerkey */

#define USAGE                                                                                                          \
  "\nUSAGE: multimap_server <multimap_server_config.yaml>\n"                                                           \
  "  multimap_server_config.yaml: Indicates which environments are going to be loaded and info on how to do it"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "multimap_server/image_loader.h"
#include "yaml-cpp/yaml.h"
#include <resource_retriever/retriever.h>
#include <ros/package.h>

#include "nav_msgs/MapMetaData.h"
#include <std_srvs/Trigger.h>
#include <multimap_server_msgs/Environment.h>
#include <multimap_server_msgs/Environments.h>
#include <multimap_server_msgs/LoadMap.h>
#include <multimap_server_msgs/DumpMap.h>
#include <multimap_server_msgs/LoadEnvironments.h>

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
void operator>>(const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class Map
{
public:
  std::string map_fullname;

  Map(const std::string& fname, const std::string& ns, const std::string& desired_name,
      const std::string& global_frame_id)
  {
    std::string mapfname = "";
    double origin[3];
    int negate;
    double occ_th, free_th;
    MapMode mode = TRINARY;
    double resolution;

    map_fullname = ns + "/" + desired_name;

    std::ifstream fin(fname.c_str());
    if (fin.fail())
    {
      ROS_ERROR("Multimap_server could not open %s.", fname.c_str());
      exit(-1);
    }
#ifdef HAVE_YAMLCPP_GT_0_5_0
    // The document loading process changed in yaml-cpp 0.5.
    YAML::Node doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif

    try
    {
      doc["resolution"] >> resolution;
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
      exit(-1);
    }
    try
    {
      doc["negate"] >> negate;
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain a negate tag or it is invalid.");
      exit(-1);
    }
    try
    {
      doc["occupied_thresh"] >> occ_th;
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
      exit(-1);
    }
    try
    {
      doc["free_thresh"] >> free_th;
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
      exit(-1);
    }
    try
    {
      std::string modeS = "";
      doc["mode"] >> modeS;

      if (modeS == "trinary")
        mode = TRINARY;
      else if (modeS == "scale")
        mode = SCALE;
      else if (modeS == "raw")
        mode = RAW;
      else
      {
        ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
        exit(-1);
      }
    }
    catch (YAML::Exception)
    {
      ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
      mode = TRINARY;
    }
    try
    {
      doc["origin"][0] >> origin[0];
      doc["origin"][1] >> origin[1];
      doc["origin"][2] >> origin[2];
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain an origin tag or it is invalid.");
      exit(-1);
    }
    try
    {
      doc["image"] >> mapfname;
      // TODO: make this path-handling more robust
      if (mapfname.size() == 0)
      {
        ROS_ERROR("The image tag cannot be an empty string.");
        exit(-1);
      }
      if (mapfname[0] != '/')
      {
        // dirname can modify what you pass it
        char* fname_copy = strdup(fname.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
      }
    }
    catch (YAML::InvalidScalar)
    {
      ROS_ERROR("The map does not contain an image tag or it is invalid.");
      exit(-1);
    }

    ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
    try
    {
      multimap_server::loadMapFromFile(&map_resp_, mapfname.c_str(), resolution, negate, occ_th, free_th, origin, mode);
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR("%s", e.what());
      exit(-1);
    }
    // To make sure get a consistent time in simulation
    ros::Time::waitForValid();
    map_resp_.map.info.map_load_time = ros::Time::now();
    map_resp_.map.header.frame_id = global_frame_id;
    map_resp_.map.header.stamp = ros::Time::now();
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell", map_resp_.map.info.width, map_resp_.map.info.height,
             map_resp_.map.info.resolution);
    meta_data_message_ = map_resp_.map.info;

    std::string service_name = ns + "/" + desired_name + "/" + "static_map";
    service = n.advertiseService(service_name, &Map::mapCallback, this);

    // Latched publisher for metadata
    std::string metadata_topic_name = ns + "/" + desired_name + "/" + "map_metadata";
    metadata_pub = n.advertise<nav_msgs::MapMetaData>(metadata_topic_name, 1, true);
    metadata_pub.publish(meta_data_message_);

    // Latched publisher for data
    std::string map_topic_name = ns + "/" + desired_name + "/" + "map";
    map_pub = n.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1, true);
    map_pub.publish(map_resp_.map);
  }

  std::string getMapFullName()
  {
    return map_fullname;
  }

private:
  ros::NodeHandle n;
  ros::Publisher map_pub;
  ros::Publisher metadata_pub;
  ros::ServiceServer service;

  /** Callback invoked when someone requests our service */
  bool mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
  {
    // request is empty; we ignore it

    // = operator is overloaded to make deep copy (tricky!)
    res = map_resp_;
    ROS_INFO("Sending map");

    return true;
  }

  /** The map data is cached here, to be sent out to service callers
   */
  nav_msgs::MapMetaData meta_data_message_;
  nav_msgs::GetMap::Response map_resp_;
};

class MultimapServer
{
public:
  /** Trivial constructor */
  MultimapServer(const std::string& fname)
  {
    timerPublish = n.createTimer(ros::Duration(0.2), &MultimapServer::timerPublishCallback, this);

    std::string load_map_service_name = "load_map";
    load_map_service = n.advertiseService(load_map_service_name, &MultimapServer::loadMapCallback, this);

    std::string load_environments_service_name = "load_environments";
    load_environments_service =
        n.advertiseService(load_environments_service_name, &MultimapServer::loadEnvironmentsCallback, this);

    std::string dump_map_service_name = "dump_map";
    dump_map_service = n.advertiseService(dump_map_service_name, &MultimapServer::dumpMapCallback, this);

    std::string dump_environments_service_name = "dump_environments";
    dump_environments_service =
        n.advertiseService(dump_environments_service_name, &MultimapServer::dumpEnvironmentsCallback, this);

    // Latched environments topic
    std::string environments_topic_name = "environments";
    environments_pub = n.advertise<multimap_server_msgs::Environments>(environments_topic_name, 1, true);

    if (false == loadEnvironmentsFromYAML(fname))
    {
      ROS_ERROR("Multimap_server could not open %s. Shutting down", fname.c_str());
      exit(-1);
    }
  }

private:
  ros::NodeHandle n;

  ros::Timer timerPublish;
  ros::Publisher environments_pub;
  ros::ServiceServer load_map_service;
  ros::ServiceServer dump_map_service;
  ros::ServiceServer load_environments_service;
  ros::ServiceServer dump_environments_service;

  std::vector<Map*> maps_vector;
  multimap_server_msgs::Environments environments_vector;

  void timerPublishCallback(const ros::TimerEvent& event)
  {
    environments_pub.publish(environments_vector);
  }

  bool loadEnvironmentsFromYAML(std::string fname)
  {
    std::ifstream fin(fname.c_str());
    if (fin.fail())
    {
      ROS_ERROR("The file %s could not be opened", fname.c_str());
      return false;
    }

#ifdef HAVE_YAMLCPP_GT_0_5_0
    // The document loading process changed in yaml-cpp 0.5.
    YAML::Node doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif

    for (YAML::const_iterator namespace_iterator = doc.begin(); namespace_iterator != doc.end(); ++namespace_iterator)
    {
      multimap_server_msgs::Environment new_environment;

      std::string global_frame = namespace_iterator->second["global_frame"].as<std::string>();
      YAML::Node maps = namespace_iterator->second["maps"];

      new_environment.name = namespace_iterator->first.as<std::string>();
      new_environment.global_frame = global_frame;

      for (YAML::const_iterator maps_iterator = maps.begin(); maps_iterator != maps.end(); ++maps_iterator)
      {
        std::string map_path = ros::package::getPath(namespace_iterator->second["maps_package"].as<std::string>()) +
                               "/" + maps_iterator->second.as<std::string>();
        std::string map_namespace = namespace_iterator->first.as<std::string>();
        std::string map_name = maps_iterator->first.as<std::string>();
        std::string map_frame = namespace_iterator->second["global_frame"].as<std::string>();

        if (isMapAlreadyLoaded(map_namespace, map_name) == true)
        {
          ROS_WARN("A map with the name %s/%s is already loaded", map_namespace.c_str(), map_name.c_str());
        }
        else
        {
          try
          {
            Map* new_map = new Map(map_path, map_namespace, map_name, map_frame);
            maps_vector.push_back(new_map);
            new_environment.map_name.push_back(map_name);
          }
          catch (std::exception& e)
          {
            ROS_WARN("load_map service failed with exception: %s", e.what());
          }
        }
      }
      environments_vector.environments.push_back(new_environment);
    }
    return true;
  }

  bool loadMapCallback(multimap_server_msgs::LoadMap::Request& req, multimap_server_msgs::LoadMap::Response& res)
  {
    std::string warning_msg = "";

    if (isMapAlreadyLoaded(req.ns, req.map_name) == true)
    {
      res.success = false;
      res.msg = "load_map service failed: a map with the same name is already loaded";
      return true;
    }

    std::ifstream fin(req.map_url.c_str());
    if (fin.fail())
    {
      res.success = false;
      res.msg = "load_map service failed: could not open %s: " + req.map_url;
      return true;
    }

    try
    {
      Map* new_map = new Map(req.map_url, req.ns, req.map_name, req.global_frame);
      maps_vector.push_back(new_map);

      bool env_exists = false;
      std::vector<multimap_server_msgs::Environment>::iterator it;
      for (it = environments_vector.environments.begin(); it != environments_vector.environments.end(); ++it)
      {
        if (it->name == req.ns)
        {
          it->map_name.push_back(req.map_name);
          env_exists = true;
          if (req.global_frame != "" && req.global_frame != it->global_frame)
          {
            warning_msg = "WARNING: You specified a global_frame, but the environment already exists with a different "
                          "global_frame. Ignoring the input value.";
            ROS_WARN("%s", warning_msg.c_str());
          }
        }
      }
      if (false == env_exists)
      {
        multimap_server_msgs::Environment new_environment;
        new_environment.name = req.ns;
        new_environment.global_frame = req.global_frame;
        new_environment.map_name.push_back(req.map_name);
        environments_vector.environments.push_back(new_environment);
      }
    }
    catch (std::exception& e)
    {
      res.success = false;
      res.msg = "load_map service failed with exception: " + std::string(e.what());
      return true;
    }

    res.success = true;
    res.msg = "load_map service worked succesfully for: " + std::string(req.map_url) + ". " + warning_msg;
    return true;
  }

  bool loadEnvironmentsCallback(multimap_server_msgs::LoadEnvironments::Request& req,
                                multimap_server_msgs::LoadEnvironments::Response& res)
  {
    if (true == loadEnvironmentsFromYAML(req.environments_url))
    {
      res.success = true;
      res.msg = "Environments loaded using file " + req.environments_url;
    }
    else
    {
      res.success = false;
      res.msg = "Multimap_server could not open " + req.environments_url;
    }
    return true;
  }

  bool dumpMapCallback(multimap_server_msgs::DumpMap::Request& req, multimap_server_msgs::DumpMap::Response& res)
  {
    bool map_deleted = false;
    bool map_deleted_from_env = false;

    std::string map_fullname = req.ns + "/" + req.map_name;

    if (isMapAlreadyLoaded(req.ns, req.map_name) == true)
    {
      std::vector<Map*>::iterator it;
      for (it = maps_vector.begin(); it != maps_vector.end(); ++it)
      {
        if ((*it)->getMapFullName() == map_fullname)
        {
          delete *it;
          it = maps_vector.erase(it);
          map_deleted = true;
        }
      }
      std::vector<multimap_server_msgs::Environment>::iterator it2;
      for (it2 = environments_vector.environments.begin(); it2 != environments_vector.environments.end(); ++it2)
      {
        if (it2->name == req.ns)
        {
          std::vector<std::string>::iterator it3;
          for (it3 = it2->map_name.begin(); it3 != it2->map_name.end();)
          {
            if (*it3 == req.map_name)
            {
              it3 = it2->map_name.erase(it3);
              // it2 = --environments_vector.environments.end();
              map_deleted_from_env = true;
            }
            else
            {
              ++it3;
            }
          }
        }
      }

      if (map_deleted && map_deleted_from_env)
      {
        res.success = true;
        res.msg = "map" + map_fullname + " removed succesfully";
        return true;
      }
      else
      {
        res.success = false;
        if (map_deleted)
        {
          res.msg = "map" + map_fullname + " deleted from maps_vector but not from environments_vector. This is a "
                                           "fatal bug, check ASAP";
        }
        else if (map_deleted_from_env)
        {
          res.msg = "map" + map_fullname + " deleted from environments_vector but not from maps_vector. This is a "
                                           "fatal bug, check ASAP";
        }
        else
        {
          res.msg = "map" + map_fullname +
                    " exists but could not be deleted from environments_vector and maps_vector. This is a "
                    "fatal bug, check ASAP";
        }
        exit(-1);
        return true;
      }
    }
    else
    {
      res.success = false;
      res.msg = "dump_map service failed: There is no map loaded under the name " + map_fullname;
      return true;
    }
  }

  // It dumps all the environments for now
  bool dumpEnvironmentsCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    std::vector<Map*>::iterator it;
    for (it = maps_vector.begin(); it != maps_vector.end(); ++it)
    {
      delete *it;
    }
    maps_vector.clear();

    environments_vector.environments.clear();

    res.success = true;
    res.message = "All environments dumped succesfully";
    return true;
  }

  bool isMapAlreadyLoaded(std::string ns, std::string map_name)
  {
    std::string map_fullname = ns + "/" + map_name;
    std::vector<Map*>::iterator it;
    for (it = maps_vector.begin(); it != maps_vector.end(); ++it)
    {
      if ((*it)->getMapFullName() == map_fullname)
      {
        return true;
      }
    }
    return false;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multimap_server", ros::init_options::AnonymousName);
  if (argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  std::string fname(argv[1]);

  try
  {
    MultimapServer ms(fname);
    ros::spin();
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("multimap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
