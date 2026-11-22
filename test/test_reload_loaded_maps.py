#!/usr/bin/env python3

import os
import time
import unittest

import rospy
import rostest
from multimap_server_msgs.msg import Environments
from multimap_server_msgs.srv import LoadEnvironments, LoadMap
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap


class ReloadLoadedMapsTest(unittest.TestCase):
    def setUp(self):
        if not rospy.core.is_initialized():
            rospy.init_node("test_reload_loaded_maps")
        self.data_dir = rospy.get_param("~data_dir")

    def _find_graph_name(self, state_index, suffix):
        deadline = time.time() + 15.0
        while time.time() < deadline and not rospy.is_shutdown():
            response = rospy.get_master().getSystemState()
            if response[0] != 1:
                self.fail("ROS master could not return its system state: {}".format(response[1]))
            entries = response[2][state_index]
            for name, _ in entries:
                if name.endswith(suffix):
                    return name
            rospy.sleep(0.1)
        self.fail("ROS graph entity not found: {}".format(suffix))

    def _load_map(self, filename, namespace, map_name, global_frame):
        service_name = self._find_graph_name(2, "/load_map")
        service = rospy.ServiceProxy(service_name, LoadMap)
        return service(
            os.path.join(self.data_dir, filename),
            namespace,
            map_name,
            global_frame,
        )

    def _load_environments(self, filename):
        service_name = self._find_graph_name(2, "/load_environments")
        service = rospy.ServiceProxy(service_name, LoadEnvironments)
        return service(os.path.join(self.data_dir, filename))

    def _get_map(self, namespace, map_name):
        suffix = "/maps/{}/{}/static_map".format(namespace, map_name)
        service_name = self._find_graph_name(2, suffix)
        response = rospy.ServiceProxy(service_name, GetMap)()
        return response.map

    def _get_environment(self, namespace, expected_names):
        topic_name = self._find_graph_name(0, "/environments")
        deadline = time.time() + 15.0
        expected_names = set(expected_names)
        while time.time() < deadline and not rospy.is_shutdown():
            message = rospy.wait_for_message(topic_name, Environments, timeout=2.0)
            matches = [
                environment
                for environment in message.environments
                if environment.name == namespace
            ]
            if len(matches) > 1:
                self.fail("Duplicate environment entries found for {}".format(namespace))
            if matches and expected_names.issubset(set(matches[0].map_name)):
                return matches[0]
            rospy.sleep(0.1)
        self.fail("Environment not found with maps: {}".format(expected_names))

    def test_environment_yaml_replaces_adds_and_preserves_maps(self):
        self.assertTrue(
            self._load_map("black.yaml", "merge_env", "replace_me", "old_map_frame").success
        )
        self.assertTrue(
            self._load_map("black.yaml", "merge_env", "keep_me", "keep_frame").success
        )

        response = self._load_environments("reload_matching.yaml")
        self.assertTrue(response.success, response.msg)

        replaced = self._get_map("merge_env", "replace_me")
        self.assertEqual(list(replaced.data), [0])
        self.assertEqual(replaced.header.frame_id, "incoming_environment_frame")
        self.assertAlmostEqual(replaced.info.resolution, 0.5)

        added = self._get_map("merge_env", "add_me")
        self.assertEqual(list(added.data), [100])
        self.assertEqual(added.header.frame_id, "incoming_environment_frame")

        kept = self._get_map("merge_env", "keep_me")
        self.assertEqual(list(kept.data), [100])
        self.assertEqual(kept.header.frame_id, "keep_frame")

        environment = self._get_environment(
            "merge_env", ["replace_me", "add_me", "keep_me"]
        )
        self.assertEqual(environment.global_frame, "environment_frame")
        self.assertEqual(environment.map_name.count("replace_me"), 1)
        self.assertEqual(environment.map_name.count("add_me"), 1)
        self.assertEqual(environment.map_name.count("keep_me"), 1)

    def test_environment_yaml_reports_failed_replacement_and_keeps_old_map(self):
        response = self._load_environments("reload_with_failure.yaml")
        self.assertFalse(response.success)
        self.assertTrue(response.msg)

        replaced = self._get_map("merge_env", "replace_me")
        self.assertEqual(list(replaced.data), [0])
        self.assertEqual(replaced.header.frame_id, "incoming_environment_frame")

        added = self._get_map("merge_env", "partial_add")
        self.assertEqual(list(added.data), [100])
        self.assertEqual(added.header.frame_id, "partial_environment_frame")

        environment = self._get_environment(
            "merge_env", ["replace_me", "add_me", "keep_me", "partial_add"]
        )
        self.assertEqual(environment.global_frame, "environment_frame")

    def test_load_map_replaces_duplicate_and_failed_reload_keeps_previous_map(self):
        first = self._load_map("black.yaml", "direct_env", "same_map", "old_frame")
        self.assertTrue(first.success, first.msg)

        replacement = self._load_map(
            "white.yaml", "direct_env", "same_map", "new_frame"
        )
        self.assertTrue(replacement.success, replacement.msg)

        loaded_map = self._get_map("direct_env", "same_map")
        self.assertEqual(list(loaded_map.data), [0])
        self.assertEqual(loaded_map.header.frame_id, "new_frame")
        self.assertAlmostEqual(loaded_map.info.resolution, 0.5)

        map_topic = self._find_graph_name(0, "/maps/direct_env/same_map/map")
        topic_map = rospy.wait_for_message(map_topic, OccupancyGrid, timeout=5.0)
        self.assertEqual(list(topic_map.data), [0])
        self.assertEqual(topic_map.header.frame_id, "new_frame")

        metadata_topic = self._find_graph_name(
            0, "/maps/direct_env/same_map/map_metadata"
        )
        metadata = rospy.wait_for_message(metadata_topic, MapMetaData, timeout=5.0)
        self.assertAlmostEqual(metadata.resolution, 0.5)

        environment = self._get_environment("direct_env", ["same_map"])
        self.assertEqual(environment.global_frame, "old_frame")
        self.assertEqual(environment.map_name.count("same_map"), 1)

        failed = self._load_map(
            "broken.yaml", "direct_env", "same_map", "failed_frame"
        )
        self.assertFalse(failed.success)
        self.assertTrue(failed.msg)

        still_loaded = self._get_map("direct_env", "same_map")
        self.assertEqual(list(still_loaded.data), [0])
        self.assertEqual(still_loaded.header.frame_id, "new_frame")

        recovered = self._load_map(
            "black.yaml", "direct_env", "same_map", "recovered_frame"
        )
        self.assertTrue(recovered.success, recovered.msg)


if __name__ == "__main__":
    rostest.rosrun("multimap_server", "reload_loaded_maps", ReloadLoadedMapsTest)
