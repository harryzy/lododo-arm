#!/usr/bin/env python3
#!/usr/bin/env python3
import json
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .yolo_detection_node import YoloDetectionNode
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject


class ArmCommandInterface(Node):
    """Listen to command topic, call YoloDetectionNode to execute related actions and publish results."""

    def __init__(self, yolo_node=None):
        super().__init__("arm_command_interface")
        self.command_sub = self.create_subscription(String, "/arm_command", self.cb_command, 10)
        self.result_pub = self.create_publisher(String, "/arm_command_result", 10)

        # Declare and read parameters from global section (/** in measurement_params.yaml)
        # Note: baseline and dual_view_timeout are GLOBAL parameters
        #       but still need to be declared to be accessible
        self.declare_parameter("baseline", 0.15)  # Default, will be overridden by global yaml
        self.declare_parameter("dual_view_timeout", 20.0)  # Default, will be overridden by global yaml
        
        self.baseline = self.get_parameter("baseline").value
        self.dual_view_timeout = self.get_parameter("dual_view_timeout").value
        
        self.get_logger().info(
            f"📊 Parameters loaded: baseline={self.baseline}, "
            f"timeout={self.dual_view_timeout}s"
        )

        # Use externally created YoloDetectionNode instance (passed from main)
        # This avoids ROS2 node name collision by ensuring yolo node is created independently
        if yolo_node is None:
            raise ValueError("yolo_node must be provided to ArmCommandInterface")
        self.yolo = yolo_node
        self.get_logger().info(f"✅ Using YoloDetectionNode: node_name={self.yolo.get_name()}, namespace={self.yolo.get_namespace()}")
        
        # CRITICAL FIX: Create subscription in ArmCommandInterface and forward to yolo_node.cb
        # This avoids subscription conflict when both nodes share same rcl node
        self._yolo_subscription = self.create_subscription(
            String,
            "/detected_objects_json",
            self._forward_to_yolo_cb,
            10
        )
        self.get_logger().info("✅ Created /detected_objects_json subscription in ArmCommandInterface")
        
        # Subscribe to collision_object to track scene objects
        # Note: /planning_scene is not actively published by move_group in this configuration
        # Instead, we subscribe to /collision_object which is published by projection_node
        self._collision_objects = {}  # dict: object_id -> CollisionObject
        self._collision_obj_sub = self.create_subscription(
            CollisionObject,
            "/collision_object",
            self._collision_object_callback,
            10
        )
        self.get_logger().info("Subscribed to /collision_object for scene object tracking")
    
    def _forward_to_yolo_cb(self, msg: String):
        """Forward /detected_objects_json messages to yolo_node.cb()"""
        self.get_logger().info(f"[FORWARD] Received message on /detected_objects_json, forwarding to yolo.cb()")
        try:
            self.yolo.cb(msg)
        except Exception as e:
            self.get_logger().error(f"[FORWARD ERROR] Failed to forward message to yolo.cb(): {e}")
    
    def _collision_object_callback(self, msg: CollisionObject):
        """Handle incoming collision objects from /collision_object topic"""
        if msg.operation == CollisionObject.ADD or msg.operation == CollisionObject.APPEND:
            self._collision_objects[msg.id] = msg
            self.get_logger().debug(f"📦 Added/Updated object: {msg.id}")
            
            # Debug: Log pose information
            if hasattr(msg, 'primitive_poses') and msg.primitive_poses:
                pose = msg.primitive_poses[0]
                self.get_logger().info(
                    f"  📍 primitive_poses[0]: X={pose.position.x:.3f}, "
                    f"Y={pose.position.y:.3f}, Z={pose.position.z:.3f}"
                )
            elif hasattr(msg, 'pose') and msg.pose:
                self.get_logger().info(
                    f"  📍 pose: X={msg.pose.position.x:.3f}, "
                    f"Y={msg.pose.position.y:.3f}, Z={msg.pose.position.z:.3f}"
                )
            else:
                self.get_logger().warn(f"  ⚠️  No pose data found in CollisionObject!")
                
        elif msg.operation == CollisionObject.REMOVE:
            if msg.id in self._collision_objects:
                del self._collision_objects[msg.id]
                self.get_logger().debug(f"🗑️  Removed object: {msg.id}")
        elif msg.operation == CollisionObject.MOVE:
            if msg.id in self._collision_objects:
                self._collision_objects[msg.id] = msg
                self.get_logger().debug(f"🔄 Moved object: {msg.id}")

    def cb_command(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {cmd}")
        if cmd == "scan_front":
            # Use tri-view intelligent scan (improve recognition rate and accuracy)
            # Thread(target=self._do_scan_front_tri_view, daemon=True).start()
            Thread(target=self._do_scan_front_planar_view, daemon=True).start()
        elif cmd == "scan_all":
            Thread(target=self._do_scan_all_traditional, daemon=True).start()
        elif cmd == "scan_and_grasp":
            # Use tri-view intelligent scan + grasp
            Thread(target=self._do_scan_and_grasp_tri_view, daemon=True).start()
        elif cmd == "hand_delivery":
            # Use tri-view intelligent scan + hand delivery
            Thread(target=self._do_hand_delivery_tri_view, daemon=True).start()
        elif cmd == "grasp_lift":
            # Use tri-view intelligent scan + grasp lift
            # Thread(target=self._do_grasp_lift_tri_view, daemon=True).start()
            Thread(target=self._do_grasp_lift_planar, daemon=True).start()
        elif cmd == "deliver_pose":
            # Use tri-view intelligent scan + specified pose delivery
        
            target_pose = self.get_scene_object_pose(timeout=2.0)
    
            if target_pose is None:
                self.get_logger().warn("get_scene_object_pose returned None, using end_effector_pose with y-offset=0.10m")
                target_pose = self.yolo.get_end_effector_pose()
                target_pose.position.y = 0.10
            
            Thread(target=self._do_deliver_pose_tri_view(target_pose), daemon=True).start()
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")
            self._publish_result({"command": cmd, "status": "unknown_command"})
    
    def _do_grasp_lift_planar(self):
        """Execute grasp lift task (using planar intelligent measurement)"""
        try:
            self.get_logger().info("🤝 Starting grasp lift task (planar mode)...")

            # Step 1: Planar intelligent scan to find target object
            self.get_logger().info("Step 1: Planar intelligent scan for target object...")
            # baseline encoding: 0.15 represents 15 degrees, need to convert to real angle
            

            results = self._do_scan_front_planar_view()
            
            if not results:
                self.get_logger().warn("Target object not found, task terminated")
                self._publish_result({
                    "command": "grasp_lift",
                    "status": "error",
                    "phase": "scan",
                    "method": "planar_intelligent_scan",
                    "error": "no_object_found"
                })
                return
            
            # Select smallest object as target (using planar-specific selector)
            target = self._select_smallest_from_planar(results)
            
            if not target:
                self.get_logger().warn("Unable to select target object, task terminated")
                self._publish_result({
                    "command": "grasp_lift",
                    "status": "error",
                    "phase": "scan",
                    "error": "target_selection_failed"
                })
                return
            
            self.get_logger().info(
                f"Found target object: {target.get('label')} "
                f"(conf={target.get('confidence', 0):.2f}, "
                f"measured={target.get('measured', False)})"
            )
            
            # Extract grasp pose
            target_pose = self._extract_grasp_pose_from_detection(target)
            
            if target_pose is None:
                self.get_logger().warn("Unable to extract grasp pose, task terminated")
                self._publish_result({
                    "command": "grasp_lift",
                    "status": "error",
                    "phase": "scan",
                    "error": "invalid_grasp_pose"
                })
                return
            
            # Phase 1 complete: publish scan result
            self._publish_result({
                "command": "grasp_lift",
                "status": "scan_complete",
                "phase": "scan",
                "method": "tri_view_triangulation",
                "object": {
                    "label": target.get("label"),
                    "confidence": target.get("confidence"),
                    "position": target.get("position"),
                    "dimensions": target.get("dimensions"),
                    "measured": target.get("measured", False),
                    "depth_error": target.get("depth_error"),
                    "size_error": target.get("size_error")
                },
                "object_pose": self._pose_to_simple(target_pose),
                "total_objects": len(results)
            })
            
            # Step 2: Execute grasp and lift
            self.get_logger().info("Step 2: Grasp object and lift...")
            grasp_success = self.yolo.grasp_and_lift(target_pose, target_info=target)
            
            if not grasp_success:
                self.get_logger().warn("Grasp failed, task terminated")
                self._publish_result({
                    "command": "grasp_lift",
                    "status": "error",
                    "phase": "grasp",
                    "method": "tri_view_triangulation",
                    "error": "grasp_failed",
                    "object": target
                })
                return
            
            # Phase 2 complete: publish grasp success result
            self.get_logger().info("✅ Grasp and lift successful!")
            self._publish_result({
                "command": "grasp_lift",
                "status": "success",
                "phase": "grasp_complete",
                "method": "tri_view_triangulation",
                "object": {
                    "label": target.get("label"),
                    "position": target.get("position"),
                    "dimensions": target.get("dimensions")
                },
                "object_pose": self._pose_to_simple(target_pose),
                "grasp_success": True
            })
            
        except Exception as e:
            self.get_logger().error(f"Grasp lift task exception: {e}")
            self._publish_result({
                "command": "grasp_lift",
                "status": "error",
                "phase": "unknown",
                "error": str(e)
            })


    def _do_grasp_lift_tri_view(self):
        """Execute grasp lift task (using tri-view intelligent measurement)"""
        try:
            self.get_logger().info("🤝 Starting grasp lift task (tri-view mode)...")

            # Step 1: Tri-view intelligent scan to find target object
            self.get_logger().info("Step 1: Tri-view intelligent scan for target object...")
            # baseline encoding: 0.15 represents 15 degrees, need to convert to real angle
            angle_degrees = self.baseline * 100.0
            results = self.yolo.scan_tri_view(
                angle_offset=angle_degrees,  # Converted angle (degrees)
                timeout=self.dual_view_timeout
            )
            
            if not results:
                self.get_logger().warn("Target object not found, task terminated")
                self._publish_result({
                    "command": "grasp_lift",
                    "status": "error",
                    "phase": "scan",
                    "method": "tri_view_triangulation",
                    "error": "no_object_found"
                })
                return
            
            # Select smallest object as target
            target = self._select_smallest_from_tri_view(results)
            
            if not target:
                self.get_logger().warn("Unable to select target object, task terminated")
                self._publish_result({
                    "command": "grasp_lift",
                    "status": "error",
                    "phase": "scan",
                    "error": "target_selection_failed"
                })
                return
            
            self.get_logger().info(
                f"Found target object: {target.get('label')} "
                f"(conf={target.get('confidence', 0):.2f}, "
                f"measured={target.get('measured', False)})"
            )
            
            # Extract grasp pose
            target_pose = self._extract_grasp_pose_from_detection(target)
            
            if target_pose is None:
                self.get_logger().warn("Unable to extract grasp pose, task terminated")
                self._publish_result({
                    "command": "grasp_lift",
                    "status": "error",
                    "phase": "scan",
                    "error": "invalid_grasp_pose"
                })
                return
            
            # Phase 1 complete: publish scan result
            self._publish_result({
                "command": "grasp_lift",
                "status": "scan_complete",
                "phase": "scan",
                "method": "tri_view_triangulation",
                "object": {
                    "label": target.get("label"),
                    "confidence": target.get("confidence"),
                    "position": target.get("position"),
                    "dimensions": target.get("dimensions"),
                    "measured": target.get("measured", False),
                    "depth_error": target.get("depth_error"),
                    "size_error": target.get("size_error")
                },
                "object_pose": self._pose_to_simple(target_pose),
                "total_objects": len(results)
            })
            
            # Step 2: Execute grasp and lift
            self.get_logger().info("Step 2: Grasp object and lift...")
            grasp_success = self.yolo.grasp_and_lift(target_pose, target_info=target)
            
            if not grasp_success:
                self.get_logger().warn("Grasp failed, task terminated")
                self._publish_result({
                    "command": "grasp_lift",
                    "status": "error",
                    "phase": "grasp",
                    "method": "tri_view_triangulation",
                    "error": "grasp_failed",
                    "object": target
                })
                return
            
            # Phase 2 complete: publish grasp success result
            self.get_logger().info("✅ Grasp and lift successful!")
            self._publish_result({
                "command": "grasp_lift",
                "status": "success",
                "phase": "grasp_complete",
                "method": "tri_view_triangulation",
                "object": {
                    "label": target.get("label"),
                    "position": target.get("position"),
                    "dimensions": target.get("dimensions")
                },
                "object_pose": self._pose_to_simple(target_pose),
                "grasp_success": True
            })
            
        except Exception as e:
            self.get_logger().error(f"Grasp lift task exception: {e}")
            self._publish_result({
                "command": "grasp_lift",
                "status": "error",
                "phase": "unknown",
                "error": str(e)
            })
    def _do_deliver_pose_tri_view(self, target_pose):
        """Execute release task"""
        try:
            self.get_logger().info("🤝 Starting release task...")

            if target_pose is None:
                self.get_logger().warn("Target pose not detected, task terminated")
                self._publish_result({
                    "command": "deliver_pose",
                    "status": "error",
                    "phase": "delivery",
                    "error": "no_target_pose_detected"
                })
                # Try to return to home position
                try:
                    if not self.yolo.go_to_home_position():
                        self.get_logger().warn("⚠️ Failed to return to home position")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Exception during home position return: {e}")
                return

            # Move above target pose and release
            self.get_logger().info("Moving above target pose and releasing object...")
            delivery_success = self.yolo.deliver_to_hand(target_pose)
            
            if delivery_success:
                self.get_logger().info("✅ Delivery task complete!")
                self._publish_result({
                    "command": "deliver_pose",
                    "status": "success",
                    "phase": "delivery_complete",
                    "delivery_pose": self._pose_to_simple(target_pose),
                    "grasp_success": True  # Add success flag
                })
            else:
                self.get_logger().warn("Delivery to target pose failed")
                self._publish_result({
                    "command": "deliver_pose",
                    "status": "error",
                    "phase": "delivery",
                    "error": "delivery_failed"
                })
                
        except Exception as e:
            self.get_logger().error(f"Delivery to target pose exception: {e}")
            self._publish_result({
                "command": "deliver_pose",
                "status": "error",
                "phase": "unknown",
                "error": str(e)
            })

    def _publish_result(self, payload: dict):
        def _sanitize(obj):
            # Recursively convert ROS Pose/Point/Quaternion and nested structures to plain Python types
            try:
                # geometry_msgs Pose
                if isinstance(obj, Pose):
                    return {
                        "position": {
                            "x": float(obj.position.x),
                            "y": float(obj.position.y),
                            "z": float(obj.position.z),
                        },
                        "orientation": {
                            "x": float(obj.orientation.x),
                            "y": float(obj.orientation.y),
                            "z": float(obj.orientation.z),
                            "w": float(obj.orientation.w),
                        },
                    }
                # Point or Quaternion
                if isinstance(obj, Point):
                    return {"x": float(obj.x), "y": float(obj.y), "z": float(obj.z)}
                if isinstance(obj, Quaternion):
                    return {"x": float(obj.x), "y": float(obj.y), "z": float(obj.z), "w": float(obj.w)}
            except Exception:
                pass

            # Duck-typing: handle Pose-like objects that aren't exact classes (e.g. wrapped msgs)
            try:
                if hasattr(obj, "position") and hasattr(obj, "orientation"):
                    p = obj.position
                    o = obj.orientation
                    return {
                        "position": {
                            "x": float(getattr(p, "x", 0.0)),
                            "y": float(getattr(p, "y", 0.0)),
                            "z": float(getattr(p, "z", 0.0)),
                        },
                        "orientation": {
                            "x": float(getattr(o, "x", 0.0)),
                            "y": float(getattr(o, "y", 0.0)),
                            "z": float(getattr(o, "z", 0.0)),
                            "w": float(getattr(o, "w", 1.0)),
                        },
                    }
            except Exception:
                pass

            # Duck-typing: simple Point-like objects
            try:
                if all(hasattr(obj, a) for a in ("x", "y", "z")):
                    return {"x": float(getattr(obj, "x")), "y": float(getattr(obj, "y")), "z": float(getattr(obj, "z"))}
            except Exception:
                pass

            if isinstance(obj, dict):
                return {k: _sanitize(v) for k, v in obj.items()}
            if isinstance(obj, (list, tuple)):
                return [_sanitize(v) for v in obj]
            # fallback for basic types
            try:
                json.dumps(obj)
                return obj
            except Exception:
                # last resort: convert to str
                return str(obj)

        try:
            s = String()
            clean = _sanitize(payload)
            s.data = json.dumps(clean)
            self.result_pub.publish(s)
            self.get_logger().info(f"Published result: {clean}")
        except Exception as e:
            self.get_logger().warn(f"Publish result failed: {e}")

    def _do_scan_front_planar_view(self):
        """Front view planar scan - Single position scan using planar detection"""
        try:
            self.get_logger().info("🔍 Executing front view planar scan...")
            
            import time
            import math
            import rclpy
            
            # Clear old scan results and enable scan mode
            # Note: With continuous detection mode, detector is always subscribed to camera
            # We just need to set scan_mode=True and clear cache
            self.yolo._scan_result = None
            self.yolo.set_scan_mode(True)
            
            results = []
            
            # Move to front position (joint1 = 0°)
            self.get_logger().info("🎥 Moving to front scan position (joint1=0°)")
            try:
                # Wait for joint_state available
                self.yolo._wait_for_joint_state()
                
                # Synchronize state immediately before movement
                self.yolo._ensure_start_state_current()
                
                # Get init_scan_pose from yolo node
                from .yolo_detection_node import _init_scan_pose
                if _init_scan_pose is not None:
                    front_cfg = list(_init_scan_pose())
                    max_abs = max(abs(a) for a in front_cfg)
                    if max_abs > 2 * math.pi:
                        front_cfg = [math.radians(a) for a in front_cfg]
                    
                    # Move to front position
                    if not self.yolo.move_to_joint_configuration(front_cfg, description="front scan position (planar)"):
                        self.get_logger().error("❌ Failed to move to front position")
                        self._publish_result({
                            "command": "scan_planar",
                            "status": "error",
                            "success": False,
                            "error": "movement_failed"
                        })
                        self.yolo.set_scan_mode(False)
                        return
                    
                    self.get_logger().info(f"✅ Moved to front position: {[f'{math.degrees(x):.1f}°' for x in front_cfg]}")
                else:
                    self.get_logger().error("❌ init_scan_pose not defined, cannot move to front position")
                    self._publish_result({
                        "command": "scan_planar",
                        "status": "error",
                        "success": False,
                        "error": "init_scan_pose_undefined"
                    })
                    self.yolo.set_scan_mode(False)
                    return
                
                # Wait for camera to stabilize
                self.yolo._sleep_with_spin(2.0)
            except Exception as e:
                self.get_logger().error(f"Move to front position failed: {e}")
                self._publish_result({
                    "command": "scan_planar",
                    "status": "error",
                    "success": False,
                    "error": str(e)
                })
                self.yolo.set_scan_mode(False)
                return
            
            # Trigger front detection
            self.yolo.trigger_detection("front")
            
            # Wait for detection result
            self.get_logger().info(f"⏳ Waiting for planar detection result (timeout {self.dual_view_timeout}s)...")
            start_time = time.time()
            result = None
            
            while time.time() - start_time < self.dual_view_timeout:
                if self.yolo._scan_result is not None:
                    result = self.yolo._scan_result
                    self.get_logger().info("✅ Planar detection complete")
                    break
                
                # Use simple sleep to avoid blocking
                time.sleep(0.1)
            
            self.yolo.set_scan_mode(False)
            
            if result is None:
                self.get_logger().warn(f"⚠️ Planar detection timeout ({self.dual_view_timeout}s)")
                self._publish_result({
                    "command": "scan_planar",
                    "status": "no_detection",
                    "success": False,
                    "method": "planar_detection",
                    "total_objects": 0,
                    "results": []
                })
                return
            
            # Extract detection results
            raw = result.get("raw")
            if isinstance(raw, dict):
                detections = raw.get("detections", [])
                results = detections  # All detected objects
                
                self.get_logger().info(f"📦 Planar scan result: total {len(results)} objects")
                for i, obj in enumerate(results):
                    self.get_logger().info(
                        f"  [{i+1}] {obj.get('label', 'unknown')}: "
                        f"conf={obj.get('confidence', 0):.2f}, "
                        f"pos=({obj.get('position', {}).get('x', 0):.3f}, "
                        f"{obj.get('position', {}).get('y', 0):.3f}, "
                        f"{obj.get('position', {}).get('z', 0):.3f})"
                    )
            
            if results:
                self._publish_result({
                    "command": "scan_planar",
                    "status": "ok",
                    "success": True,
                    "method": "planar_detection",
                    "total_objects": len(results),
                    "results": results
                })
                return results
            else:
                self.get_logger().warn("Planar scan detected no objects")
                self._publish_result({
                    "command": "scan_planar",
                    "status": "no_detection",
                    "success": False,
                    "method": "planar_detection",
                    "total_objects": 0,
                    "results": []
                })
                
        except Exception as e:
            self.get_logger().error(f"Planar scan failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self._publish_result({
                "command": "scan_planar",
                "status": "error",
                "success": False,
                "error": str(e)
            })
            # Ensure scan mode is disabled
            try:
                self.yolo.set_scan_mode(False)
            except:
                pass

    def _do_scan_front_tri_view(self):
        """Front view tri-view intelligent scan - Improve recognition rate and accuracy"""
        try:
            self.get_logger().info("🔍 Executing front view tri-view intelligent scan...")
            
            # Calling tri-view scan
            results = self.yolo.scan_tri_view(
                angle_offset=15.0,  # view angle offset
                timeout=self.dual_view_timeout
            )
            
            if results:
                # Find smallest target
                smallest = self._select_smallest_from_tri_view(results)
                
                # Add debug info
                self.get_logger().info(f"📦 scan result: total{len(results)} objects")
                for i, obj in enumerate(results):
                    self.get_logger().info(
                        f"  [{i+1}] {obj.get('label', 'unknown')}: "
                        f"measured={obj.get('measured', False)}, "
                        f"has_dimensions={obj.get('dimensions') is not None}"
                    )
                
                self._publish_result({
                    "command": "scan_tri",
                    "status": "ok",
                    "success": True,
                    "method": "tri_view_triangulation",  # Tri-view identifier
                    "total_objects": len(results),
                    "results": results,
                    "smallest_target": smallest.get("label") if smallest else None
                })
            else:
                self.get_logger().warn("Tri-view scan detected no objects")
                self._publish_result({
                    "command": "scan_tri",
                    "status": "no_detection",
                    "success": False,
                    "method": "tri_view_triangulation",
                    "total_objects": 0,
                    "results": []
                })
                
        except Exception as e:
            self.get_logger().error(f"Tri-view scan failed: {e}")
            self._publish_result({
                "command": "scan_tri",
                "status": "error",
                "success": False,
                "error": str(e)
            })

    def _do_scan_all_traditional(self):
        """Traditional multi-view scan (no dual-view triangulation measurement)"""
        try:
            self.get_logger().info("🔄 Executing traditional multi-view scan...")
            
            # Using traditional scan_views method
            results = self.yolo.scan_views(
                stop_on_detection=False, 
                front_only=False
            )
            
            # Check if any view successfully detected objects
            has_detection = any(view.get("success", False) for view in results)
            
            # Collect all detected objects
            all_objects = []
            for view in results:
                if view.get("success") and view.get("poses"):
                    all_objects.extend(view.get("poses", []))
            
            self._publish_result({
                "command": "scan_all",
                "status": "ok",
                "success": has_detection,  # Add success field for RVIZ plugin recognition
                "method": "traditional_multi_view",
                "total_objects": len(all_objects),
                "views_scanned": len(results),
                "results": results,
                "objects": all_objects  # Object list from all views
            })
            
        except Exception as e:
            self.get_logger().error(f"Traditional multi-view scan failed: {e}")
            self._publish_result({
                "command": "scan_all",
                "status": "error",
                "success": False,  # Add success field
                "error": str(e)
            })

    def _do_scan_and_grasp_tri_view(self):
        """Tri-view intelligent scan and grasp - Improve recognition rate and accuracy"""
        try:
            self.get_logger().info("🎯 Executing tri-view intelligent scan and grasp...")
            
            # Tri-view intelligent scan
            # baseline encoding: 0.15 represents 15 degrees, need to convert to real angle
            angle_degrees = self.baseline * 100.0
            results = self.yolo.scan_tri_view(
                angle_offset=angle_degrees,  # Converted angle (degrees)
                timeout=self.dual_view_timeout
            )
            
            if not results:
                self.get_logger().warn("Tri-view scan detected no objects")
                self._publish_result({
                    "command": "scan_and_grasp",
                    "status": "no_target",
                    "method": "tri_view_triangulation",
                    "results": []
                })
                return
            
            # Selected smallest object
            target = self._select_smallest_from_tri_view(results)
            
            if not target:
                self._publish_result({
                    "command": "scan_and_grasp",
                    "status": "no_target",
                    "results": results
                })
                return
            
            # Extract grasp pose
            grasp_pose = self._extract_grasp_pose_from_detection(target)
            
            if grasp_pose is None:
                self._publish_result({
                    "command": "scan_and_grasp",
                    "status": "invalid_pose",
                    "target": target
                })
                return
            
            found = {
                "label": target.get("label"),
                "position": target.get("position"),
                "dimensions": target.get("dimensions"),
                "measured": target.get("measured", False),
                "depth_error": target.get("depth_error"),
                "pose": self._pose_to_simple(grasp_pose),
                "view_pair": target.get("view_pair", "unknown")  # Record used view pair
            }
            
            # Executing grasp
            try:
                ok = self.yolo.grasping_for_scan_rs(grasp_pose, None)
            except Exception as e:
                self.get_logger().warn(f"Grasp call failed: {e}")
                ok = False
            
            self._publish_result({
                "command": "scan_and_grasp",
                "status": "done",
                "method": "tri_view_triangulation",
                "found": found,
                "grasp_success": bool(ok)
            })
            
        except Exception as e:
            self.get_logger().error(f"Tri-view scan and grasp failed: {e}")
            self._publish_result({
                "command": "scan_and_grasp",
                "status": "error",
                "error": str(e)
            })

    def _pose_to_simple(self, pose):
        try:
            return {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z}
        except Exception:
            return None

    def _select_best_target(self, results: list, target_label: str = None) -> tuple:
        """
        Select the best target from scan results.

        Strategy:
        1. If target_label is specified, prioritize selecting this type of object
        2. If not specified or specified type not found, select the object with the smallest physical size

        Note: Since bbox_px is pixel size, it is greatly affected by object distance. Therefore, depth normalization is needed
        to estimate the real physical size. Normalized size = pixel size × depth distance.

        :param results: Result list returned by scan_views
        :param target_label: Optional target label (such as "bottle", "cup", etc)
        :return: (target_pose, view_name, detection_info) if target found, otherwise (None, None, None)
        """
        import math
        
        best_target = None
        best_size = float('inf')
        best_view = None
        best_detection_info = None
        
        for view in results:
            view_cmd = view.get("view_cmd")
            poses = view.get("poses") or []
            detections = view.get("detections") or []
            
            # If no detection result, skip
            if not poses or not detections:
                continue
            
            # Iterate through all detection results
            for i, detection in enumerate(detections):
                if i >= len(poses):
                    break
                
                pose = poses[i]
                label = detection.get("label") or detection.get("class_name") or ""
                
                # If target label specified, check if matches
                if target_label:
                    if label.lower() != target_label.lower():
                        continue
                
                # Get bbox pixel size (needed in all cases)
                bbox = detection.get("bbox_px") or {}
                width_px = bbox.get("w", 0)
                height_px = bbox.get("h", 0)
                
                # Get object position for depth calculation
                position = detection.get("position") or {}
                x = position.get("x", 0)
                y = position.get("y", 0)
                
                # Check if x and y are valid numbers (not NaN or None)
                if x is None or y is None or math.isnan(x) or math.isnan(y):
                    # If position invalid, use bbox size for rough depth estimation
                    # This is a rough estimate, but better than using fixed value
                    if width_px > 0 and height_px > 0:
                        # Roughly estimate distance based on bbox size in image
                        # Larger pixels mean closer object; smaller pixels mean farther object
                        # Assuming average pixel size 200px corresponds to about 0.5 meter distance
                        avg_px_size = math.sqrt(width_px * height_px)
                        depth = max(0.3, min(1.5, 100.0 / avg_px_size))
                        self.get_logger().warn(
                            f"Object position invalid (NaN), using bbox to estimate depth: "
                            f"label={label}, bbox={width_px:.0f}×{height_px:.0f}px, "
                            f"est_depth={depth:.2f}m"
                        )
                    else:
                        depth = 0.5  # Default 0.5 meters
                        self.get_logger().warn(
                            f"Both object position and size invalid, using default depth: label={label}, depth={depth}m"
                        )
                else:
                    # Calculate horizontal distance to robot (depth)
                    depth = math.sqrt(x*x + y*y)
                    
                    # If depth too small or 0, use default value to avoid division by zero error
                    if depth < 0.1 or math.isnan(depth):
                        depth = 0.5  # Default 0.5 meters
                
                # Use depth-normalized pixel size to estimate physical size
                # Normalized size ≈ pixel size × depth / focal length estimation coefficient
                # Simplified here as: normalized_size = pixel_area × depth²
                # This allows fair comparison of objects at different distances
                if width_px > 0 and height_px > 0:
                    # Use square of depth for normalization, because object size projection in image is inversely proportional to square of distance
                    normalized_size = (width_px * height_px) * (depth ** 2)
                else:
                    # If no size info, use a default large size value
                    normalized_size = float('inf')
                
                # Select object with smallest normalized size
                if normalized_size < best_size:
                    best_size = normalized_size
                    best_target = pose
                    best_view = view_cmd
                    best_detection_info = {
                        "label": label,
                        "confidence": detection.get("confidence"),
                        "bbox_px": bbox,
                        "position": detection.get("position"),
                        "depth": depth,
                        "normalized_size": normalized_size
                    }
                    
                    self.get_logger().info(
                        f"Found candidate target: label={label}, "
                        f"bbox={width_px:.0f}×{height_px:.0f}px, depth={depth:.2f}m, "
                        f"norm_size={normalized_size:.1f}, view={view_cmd}, "
                        f"conf={detection.get('confidence', 0):.2f}"
                    )
        
        if best_target:
            self.get_logger().info(
                f"Selected best target: label={best_detection_info['label']}, "
                f"norm_size={best_size:.1f}, depth={best_detection_info.get('depth', 0):.2f}m, "
                f"view={best_view}"
            )
        
        return best_target, best_view, best_detection_info

    def _do_hand_delivery_tri_view(self):
        """Execute hand delivery task (using tri-view intelligent measurement)"""
        try:
            self.get_logger().info("🤝 Starting hand delivery task (tri-view mode)...")
            
            # Step 1: Tri-view intelligent scan to find target object
            self.get_logger().info("Step 1: Tri-view intelligent scan for target object...")
            # baseline encoding: 0.15 represents 15 degrees, need to convert to real angle
            angle_degrees = self.baseline * 100.0
            results = self.yolo.scan_tri_view(
                angle_offset=angle_degrees,  # Converted angle (degrees)
                timeout=self.dual_view_timeout
            )
            
            if not results:
                self.get_logger().warn("Target object not found, task terminated")
                self._publish_result({
                    "command": "hand_delivery",
                    "status": "error",
                    "method": "tri_view_triangulation",
                    "error": "no_object_found"
                })
                return

            # Select the smallest object as target
            target = self._select_smallest_from_tri_view(results)

            if not target:
                self.get_logger().warn("Unable to select target object, task terminated")
                self._publish_result({
                    "command": "hand_delivery",
                    "status": "error",
                    "error": "target_selection_failed"
                })
                return
            
            self.get_logger().info(
                f"Found target object: {target.get('label')} "
                f"(conf={target.get('confidence', 0):.2f}, "
                f"measured={target.get('measured', False)})"
            )
            
            # Extract grasp pose
            target_pose = self._extract_grasp_pose_from_detection(target)
            
            if target_pose is None:
                self.get_logger().warn("Unable to extract grasp pose，task terminated")
                self._publish_result({
                    "command": "hand_delivery",
                    "status": "error",
                    "phase": "scan",
                    "error": "invalid_grasp_pose"
                })
                return
            
            # Phase 1 complete: publish scan result
            self._publish_result({
                "command": "hand_delivery",
                "status": "scan_complete",
                "phase": "scan",
                "method": "tri_view_triangulation",
                "object": {
                    "label": target.get("label"),
                    "confidence": target.get("confidence"),
                    "position": target.get("position"),
                    "dimensions": target.get("dimensions"),
                    "measured": target.get("measured", False),
                    "depth_error": target.get("depth_error"),
                    "size_error": target.get("size_error")
                },
                "object_pose": self._pose_to_simple(target_pose),
                "total_objects": len(results)
            })
            
            # Step 2: Execute grasp and lift
            self.get_logger().info("Step 2: Grasp object and lift...")
            grasp_success = self.yolo.grasp_and_lift(target_pose, target_info=target)
            
            if not grasp_success:
                self.get_logger().warn("Grasp failed, task terminated")
                self._publish_result({
                    "command": "hand_delivery",
                    "status": "error",
                    "phase": "grasp",
                    "method": "tri_view_triangulation",
                    "error": "grasp_failed",
                    "object": target
                })
                return
            
            # Phase 2 complete: publish grasp success result
            self._publish_result({
                "command": "hand_delivery",
                "status": "grasp_complete",
                "phase": "grasp",
                "method": "tri_view_triangulation",
                "object": {
                    "label": target.get("label"),
                    "position": target.get("position"),
                    "dimensions": target.get("dimensions")
                },
                "object_pose": self._pose_to_simple(target_pose),
                "grasp_success": True
            })
            
            # Step 3: Waiting to detect hand
            self.get_logger().info("Step 3: Waiting to detect hand position...")
            hand_pose = self.yolo.wait_for_hand_detection(timeout=30.0)
            
            if hand_pose is None:
                self.get_logger().warn("Hand not detected, task terminated")
                self._publish_result({
                    "command": "hand_delivery",
                    "status": "error",
                    "phase": "hand_detection",
                    "error": "no_hand_detected"
                })
                # Try to return to home position
                try:
                    if not self.yolo.go_to_home_position():
                        self.get_logger().warn("⚠️ Failed to return to home position")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Exception during home position return: {e}")
                return
            
            # Step 4: Move above hand and release
            self.get_logger().info("Step 4: Moving above hand and releasing object...")
            delivery_success = self.yolo.deliver_to_hand(hand_pose)
            
            if delivery_success:
                self.get_logger().info("✅ Hand delivery task complete!")
                self._publish_result({
                    "command": "hand_delivery",
                    "status": "success",
                    "phase": "delivery_complete",
                    "method": "tri_view_triangulation",
                    "object": {
                        "label": target.get("label"),
                        "position": target.get("position"),
                        "dimensions": target.get("dimensions"),
                        "measured": target.get("measured", False)
                    },
                    "object_pose": self._pose_to_simple(target_pose),
                    "hand_pose": self._pose_to_simple(hand_pose)
                })
            else:
                self.get_logger().warn("Delivery to hand failed")
                self._publish_result({
                    "command": "hand_delivery",
                    "status": "error",
                    "phase": "delivery",
                    "error": "delivery_failed"
                })
                
        except Exception as e:
            self.get_logger().error(f"Hand delivery task exception: {e}")
            self._publish_result({
                "command": "hand_delivery",
                "status": "error",
                "error": str(e)
            })

    def _select_smallest_from_planar(self, results: list) -> dict:
        """
        Select the smallest object from planar detection results.

        :param results: Result list from planar detection (contains bbox_px)
        :return: Detection info of the smallest object, return None if none
        """
        if not results:
            return None
        
        # If only one object, return it directly
        if len(results) == 1:
            return results[0]
        
        # Multiple objects: select by smallest bounding box area
        smallest = None
        smallest_area = float('inf')
        
        for obj in results:
            bbox = obj.get("bbox_px")
            if not bbox:
                continue
            
            w = bbox.get("w", 0)
            h = bbox.get("h", 0)
            
            if w > 0 and h > 0:
                area = w * h
                if area < smallest_area:
                    smallest_area = area
                    smallest = obj
        
        return smallest

    def _select_smallest_from_tri_view(self, results: list) -> dict:
        """
        Select the smallest object from tri-view measurement results.

        :param results: Result list returned by scan_tri_view
        :return: Detection info of the smallest object, return None if none
        """
        import math
        
        if not results:
            return None
        
        smallest = None
        smallest_volume = float('inf')
        
        for obj in results:
            # Get size info
            dims = obj.get("dimensions")
            if not dims:
                continue
            
            width = dims.get("width", 0)
            depth = dims.get("depth", 0)
            height = dims.get("height", 0)
            
            # Calculate volume
            if width > 0 and depth > 0 and height > 0:
                volume = width * depth * height
                
                if volume < smallest_volume:
                    smallest_volume = volume
                    smallest = obj
                    
                    self.get_logger().info(
                        f"Candidate object: {obj.get('label')}, "
                        f"size={width*1000:.1f}×{depth*1000:.1f}×{height*1000:.1f}mm, "
                        f"volume={volume*1e9:.1f}cm³"
                    )
        
        if smallest:
            self.get_logger().info(
                f"✅ Selected smallest object: {smallest.get('label')}, "
                f"volume={smallest_volume*1e9:.1f}cm³"
            )
        
        return smallest

    def _extract_grasp_pose_from_detection(self, detection: dict):
        """
        Extract grasp pose from detection result
        
        :param detection: Detection info dictionary
        :return: Pose object, return None if extraction failed
        """
        from geometry_msgs.msg import Pose, Point, Quaternion
        
        # Prioritize using grasp_pose
        grasp_info = detection.get("grasp_pose")
        if grasp_info:
            try:
                pose = Pose()
                
                # Extract position
                pos = grasp_info.get("position")
                if pos:
                    pose.position.x = float(pos.get("x", 0))
                    pose.position.y = float(pos.get("y", 0))
                    pose.position.z = float(pos.get("z", 0))
                
                # Extract pose
                ori = grasp_info.get("orientation")
                if ori:
                    pose.orientation.x = float(ori.get("x", 0))
                    pose.orientation.y = float(ori.get("y", 0))
                    pose.orientation.z = float(ori.get("z", 0))
                    pose.orientation.w = float(ori.get("w", 1))
                else:
                    # Default pose
                    pose.orientation.w = 1.0
                
                return pose
            except Exception as e:
                self.get_logger().warn(f"Extraction from grasp_pose failed: {e}")
        
        # Fallback: Use object center position
        pos = detection.get("position")
        if pos:
            try:
                pose = Pose()
                pose.position.x = float(pos.get("x", 0))
                pose.position.y = float(pos.get("y", 0))
                pose.position.z = float(pos.get("z", 0))
                pose.orientation.w = 1.0
                
                self.get_logger().info("Using object center position as grasp point")
                return pose
            except Exception as e:
                self.get_logger().warn(f"Extraction from position failed: {e}")
        
        return None

    def get_scene_object_pose(self, timeout: float = 2.0) -> Pose:
        """
        Get pose of scene object from collision objects
        
        Strategy:
        - If only 1 object: return its pose
        - If multiple objects: return pose of object with X closest to 0 (closest to robot base)
        - If no objects: return None
        
        Args:
            timeout: Maximum wait time for collision object data (seconds)
            
        Returns:
            Pose object if found, None otherwise
        """
        import time
        
        # Wait for collision object data
        start_time = time.time()
        while not self._collision_objects:
            if time.time() - start_time > timeout:
                self.get_logger().warn(
                    f"⏱️  Timeout waiting for collision object data ({timeout}s)"
                )
                return None
            
            # Keep ROS spinning to receive messages
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except Exception:
                pass
            
            time.sleep(0.05)
        
        # Get all collision objects
        objects = list(self._collision_objects.values())
        num_objects = len(objects)
        
        if num_objects == 0:
            self.get_logger().warn("⚠️  No collision objects available")
            return None
        
        self.get_logger().info(f"📦 Found {num_objects} scene object(s)")
        
        # Case 1: Only one object - return its pose directly
        if num_objects == 1:
            obj = objects[0]
            pose = self._extract_pose_from_collision_object(obj)
            
            if pose:
                self.get_logger().info(
                    f"✅ Single object found: '{obj.id}' at "
                    f"X={pose.position.x:.3f}, Y={pose.position.y:.3f}, Z={pose.position.z:.3f}"
                )
                return pose
            else:
                self.get_logger().warn(f"⚠️  Object '{obj.id}' has no pose data")
                return None
        
        # Case 2: Multiple objects - find one with X closest to 0
        closest_obj = None
        closest_pose = None
        min_x_distance = float('inf')
        
        for obj in objects:
            # Extract pose from collision object
            pose = self._extract_pose_from_collision_object(obj)
            
            if pose is None:
                self.get_logger().warn(f"⚠️  Object '{obj.id}' has no pose data, skipping")
                continue
            
            # Calculate distance from X=0 (robot base)
            x_distance = abs(pose.position.x)
            
            self.get_logger().info(
                f"  Object '{obj.id}': "
                f"X={pose.position.x:.3f}, Y={pose.position.y:.3f}, Z={pose.position.z:.3f}, "
                f"|X|={x_distance:.3f}"
            )
            
            if x_distance < min_x_distance:
                min_x_distance = x_distance
                closest_obj = obj
                closest_pose = pose
        
        if closest_obj:
            self.get_logger().info(
                f"✅ Selected closest object: '{closest_obj.id}' "
                f"(|X|={min_x_distance:.3f}m from base)"
            )
            return closest_pose
        
        self.get_logger().warn("⚠️  No valid object poses found")
        return None
    
    def _extract_pose_from_collision_object(self, obj: CollisionObject) -> Pose:
        """
        Extract pose from CollisionObject
        
        Tries multiple fields in order:
        1. obj.pose (direct pose field)
        2. obj.primitive_poses[0] (for primitives like boxes)
        3. obj.mesh_poses[0] (for mesh objects)
        
        Returns:
            Pose object if found, None otherwise
        """
        self.get_logger().info(f"🔍 Extracting pose from object '{obj.id}'...")
        
        try:
            # Try direct pose field first
            if hasattr(obj, 'pose') and obj.pose is not None:
                # Check if pose has valid position (not all zeros)
                if obj.pose.position.x != 0.0 or obj.pose.position.y != 0.0 or obj.pose.position.z != 0.0:
                    self.get_logger().info(
                        f"  ✅ Found pose field: X={obj.pose.position.x:.3f}, "
                        f"Y={obj.pose.position.y:.3f}, Z={obj.pose.position.z:.3f}"
                    )
                    return obj.pose
                else:
                    self.get_logger().warn("  ⚠️  pose field exists but is (0,0,0)")
            
            # Try primitive_poses
            if hasattr(obj, 'primitive_poses') and obj.primitive_poses:
                self.get_logger().info(f"  📦 Found {len(obj.primitive_poses)} primitive_poses")
                pose = obj.primitive_poses[0]
                self.get_logger().info(
                    f"  ✅ Using primitive_poses[0]: X={pose.position.x:.3f}, "
                    f"Y={pose.position.y:.3f}, Z={pose.position.z:.3f}"
                )
                return pose
            
            # Try mesh_poses
            if hasattr(obj, 'mesh_poses') and obj.mesh_poses:
                self.get_logger().info(f"  🔺 Found {len(obj.mesh_poses)} mesh_poses")
                return obj.mesh_poses[0]
            
            # Try subframe_poses (less common)
            if hasattr(obj, 'subframe_poses') and obj.subframe_poses:
                self.get_logger().info(f"  🔲 Found {len(obj.subframe_poses)} subframe_poses")
                return obj.subframe_poses[0]
            
            self.get_logger().error(f"  ❌ No pose data found in any field!")
                
        except Exception as e:
            self.get_logger().error(f"  ❌ Error extracting pose from object '{obj.id}': {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        return None



def main(args=None):
    rclpy.init(args=args)
    node = None
    yolo_node = None
    executor = None
    
    try:
        # SOLUTION: Create YoloDetectionNode without subscription
        # Subscription will be created by ArmCommandInterface and forwarded
        # This avoids subscription conflict when both objects share same rcl node
        yolo_node = YoloDetectionNode(
            node_name="yolo_detection_node",
            is_fast_robust_plan=True,
            create_subscription=False  # Subscription handled by ArmCommandInterface
        )
        
        # Check if node was created with correct name
        actual_name = yolo_node.get_name()
        if actual_name != "yolo_detection_node":
            yolo_node.get_logger().info(
                f"ℹ️ YoloDetectionNode shares node name '{actual_name}' with ArmCommandInterface. "
                f"Subscription forwarding pattern is active."
            )
        else:
            yolo_node.get_logger().info(f"✅ YoloDetectionNode created with name: {actual_name}")
        
        # Create ArmCommandInterface and pass the yolo_node to it
        # ArmCommandInterface will create the subscription and forward messages
        node = ArmCommandInterface(yolo_node=yolo_node)
        node.get_logger().info(f"✅ ArmCommandInterface created with name: {node.get_name()}")

        # Create a SingleThreadedExecutor and add both nodes to it to avoid multiple rclpy.spin calls
        import rclpy.executors as rex

        ExecST = getattr(rex, "SingleThreadedExecutor", None)
        if ExecST is None:
            raise RuntimeError("SingleThreadedExecutor not available")
        executor = ExecST(context=node.context)
        executor.add_node(node)
        try:
            executor.add_node(yolo_node)
        except Exception:
            node.get_logger().warn("Unable to add internal yolo node to executor")

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    except Exception as e:
        # fallback to simple spin if executor couldn't be created
        if node:
            node.get_logger().error(f"Executor creation/execution failed: {e}; Falling back to rclpy.spin(node)")
            try:
                rclpy.spin(node)
            except Exception:
                pass
        else:
            print(f"Executor creation failed and node not created: {e}")
    finally:
        # Cleanup: shutdown executor and remove node
        try:
            if executor is not None:
                    executor.shutdown()
                    executor.remove_node(node)
                    if yolo_node is not None:
                        executor.remove_node(yolo_node)
            if node:
                node.get_logger().info("Destroying node and exiting...")
            if yolo_node is not None:
                yolo_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
