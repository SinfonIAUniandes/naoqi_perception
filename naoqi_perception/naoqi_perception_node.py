import rclpy
from rclpy.node import Node
import qi
import argparse
import sys
import time

from naoqi_utilities_msgs.srv import PointAt, SetTrackerMode

class NaoqiPerceptionNode(Node):
    """
    ROS2 Node to manage perception functionalities of a NAO robot, like tracking.
    """
    def __init__(self, session):
        """
        Initializes the node, NAOqi service clients, and ROS2 services.
        """
        super().__init__('naoqi_perception_node')
        self.get_logger().info("Initializing NaoqiPerceptionNode...")

        # --- NAOqi Service Clients ---
        try:
            self.al_tracker = session.service("ALTracker")
            self.al_motion = session.service("ALMotion") # Needed for some tracker operations
            self.al_basic_awareness = session.service("ALBasicAwareness")
            self.get_logger().info("NAOqi service clients obtained successfully.")
        except Exception as e:
            self.get_logger().error(f"Could not connect to NAOqi services: {e}")
            sys.exit(1)

        # --- ROS2 Services for ALTracker ---
        self.point_at_service = self.create_service(
            PointAt,
            '~/point_at',
            self.point_at_callback
        )
        self.set_tracker_mode_service = self.create_service(
            SetTrackerMode,
            '~/set_tracker_mode',
            self.set_tracker_mode_callback
        )

        self.get_logger().info("Perception functionalities node is ready.")

    def point_at_callback(self, request, response):
        """
        Callback to make the robot point at a specific coordinate.
        """
        try:
            self.get_logger().info(f"Request to point at {request.point} with effector '{request.effector_name}'.")
            point = [request.point.x, request.point.y, request.point.z]
            self.al_tracker.pointAt(request.effector_name, point, request.frame, request.speed)
        except Exception as e:
            self.get_logger().error(f"Error pointing at target: {e}")
        return response

    def set_tracker_mode_callback(self, request, response):
        """
        Callback to set the tracker mode.
        Modes: "stop", "start_head", "start_move", "start_body_rotation"
        """
        try:
            if request.mode == "stop":
                self.al_basic_awareness.pauseAwareness()
                self.al_basic_awareness.stopAwareness()
                time.sleep(1)
                self.get_logger().info("Request to stop tracker.")
                self.al_motion.setAngles(["HeadPitch", "HeadYaw"], [0.0, 0.0], 0.2) # Move head to default
                time.sleep(1)
                self.al_tracker.setMaximumDistanceDetection(0.1)
                self.al_tracker.setEffector("None")
                self.al_tracker.stopTracker()
                time.sleep(1)
                self.al_tracker.unregisterAllTargets()
                response.success = True
                response.message = "Tracker stopped successfully."
            elif request.mode == "start_head":
                self.al_basic_awareness.pauseAwareness()
                self.al_basic_awareness.stopAwareness()
                time.sleep(1)
                self.get_logger().info("Request to start tracker for 'Face' (head only).")
                self.al_motion.setAngles(["HeadPitch", "HeadYaw"], [0.0, 0.0], 0.2) # Move head to default
                self.al_tracker.setEffector("None")
                targetName = "Face"
                faceWidth = 0.1
                self.al_tracker.registerTarget(targetName, faceWidth)
                self.al_tracker.setRelativePosition([0.3, 0.0, 0.0,
                                                        0.1, 0.1, 0.3])
                self.al_tracker.setMaximumDistanceDetection(3.5)
                self.al_tracker.setMode("Head")
                self.al_tracker.track(targetName)
                self.al_tracker.initialize()
                response.success = True
                response.message = "Tracker started for 'Face'."
            elif request.mode == "start_body_rotation":
                self.get_logger().info("Request to start tracking of sound and faces rotating")
                self.al_basic_awareness.stopAwareness()
                time.sleep(1)
                self.al_basic_awareness.startAwareness()
                time.sleep(1)
                self.al_basic_awareness.pauseAwareness()
                time.sleep(1)
                self.al_basic_awareness.resumeAwareness()
                time.sleep(1)
                self.al_basic_awareness.setTrackingMode("BodyRotation")
                response.success = True
                response.message = "Face tracking with movement enabled."
                self.get_logger().info(response.message)
            elif request.mode == "start_move":
                self.al_basic_awareness.pauseAwareness()
                self.al_basic_awareness.stopAwareness()
                time.sleep(1)
                self.get_logger().info("Request to start face tracking (with movement).")
                self.al_motion.setAngles(["HeadPitch", "HeadYaw"], [0.0, 0.0], 0.2)
                self.al_tracker.setMaximumDistanceDetection(3.5)
                self.al_tracker.initialize()
                self.al_tracker.setMode("Move")
                self.al_tracker.registerTarget("Face", 0.2)
                self.al_tracker.track("Face")
                response.success = True
                response.message = "Face tracking with movement enabled."
            else:
                response.success = False
                response.message = f"Invalid mode '{request.mode}'. Use 'stop', 'start_head', or 'start_move'."
                self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Error setting tracker mode '{request.mode}': {e}"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On Robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    parsed_args, _ = parser.parse_known_args(args=sys.argv[1:])

    session = qi.Session()
    try:
        session.connect(f"tcp://{parsed_args.ip}:{parsed_args.port}")
    except RuntimeError:
        print(f"Can't connect to Naoqi at ip \"{parsed_args.ip}\" on port {parsed_args.port}.\n"
              "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    naoqi_perception_node = NaoqiPerceptionNode(session)

    try:
        rclpy.spin(naoqi_perception_node)
    except KeyboardInterrupt:
        print("Closing the perception functionalities node.")
    finally:
        naoqi_perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()