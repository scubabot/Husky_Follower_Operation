#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from zed_msgs.msg import ObjectsStamped 

class ZedPersonFollower(Node):
    def __init__(self):
        super().__init__('zed_person_follower')
        self.cmd_vel_pub = self.create_publisher(Twist, '/a200_1046/cmd_vel', 10)
        self.sub = self.create_subscription(ObjectsStamped, '/zed/zed_node/obj_det/objects', self.object_callback, 10)

        # --- Parameters ---
        self.declare_parameter('target_dist', 0.5)      
        self.declare_parameter('mounting_offset', 0.5) 
        
        # --- PIXEL TRACKING CONFIG ---
        # Assuming ZED X is 1920x1080. Center is 960.
        self.image_center_x = 960.0 
        
        # --- GAINS ---
        self.kp_lin = 0.65  
        self.kd_lin = 0.25  
        # Pixel gains are different scale. 0.001 is a good starting point.
        self.kp_ang_pixel = 0.0015 
        self.kd_ang_pixel = 0.0002 

        self.alpha = 0.25    
        self.prev_lin_error = 0.0
        self.prev_pixel_error = 0.0
        self.filtered_depth = 0.0
        self.last_v = 0.0  
        self.last_time = self.get_clock().now()

        # Search Config
        self.search_angular_vel = 0.4 
        self.lost_timeout = 1.5       
        self.last_seen_time = self.get_clock().now()

        self.get_logger().info("V9: Centroid Tracking Active. Aiming for image center.")

    def object_callback(self, msg):
        curr_time = self.get_clock().now()
        valid_person = [obj for obj in msg.objects if obj.label.upper() == "PERSON"]

        if not valid_person:
            if (curr_time - self.last_seen_time).nanoseconds / 1e9 > self.lost_timeout:
                self.perform_search()
            else:
                self.stop_robot()
            return

        self.last_seen_time = curr_time
        target = valid_person[0]
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0: return

        # 1. DEPTH (X-axis per your ZED config)
        raw_depth = target.position[0] 
        self.filtered_depth = (self.alpha * raw_depth) + (1.0 - self.alpha) * self.filtered_depth
        lin_error = self.filtered_depth - self.get_parameter('target_dist').value

        # 2. PIXEL CENTERING (Steering)
        # Calculate horizontal center of the 2D bounding box
        # corners are: [top-left, top-right, bottom-right, bottom-left]
        corners_x = [c.kp[0] for c in target.bounding_box_2d.corners]
        bbox_center_x = sum(corners_x) / len(corners_x)
        
        # Error in pixels (Positive means person is to the right of center)
        pixel_error = self.image_center_x - bbox_center_x

        # 3. CONTROL LAWS
        # Linear (PD)
        lin_deriv = (lin_error - self.prev_lin_error) / dt
        v_raw = (self.kp_lin * lin_error) + (self.kd_lin * lin_deriv)

        # Angular (PD using pixels)
        pix_deriv = (pixel_error - self.prev_pixel_error) / dt
        # We want to turn toward the error
        w = (self.kp_ang_pixel * pixel_error) + (self.kd_ang_pixel * pix_deriv)

        # 4. OUTPUTS
        v_limited = self.last_v + max(min(v_raw - self.last_v, 0.8 * dt), -0.8 * dt)
        
        drive_msg = Twist()
        if abs(lin_error) > 0.06:
            drive_msg.linear.x = float(max(min(v_limited, 0.45), -0.45))
        
        # Soft-cap for steering to prevent the 90-degree snap
        drive_msg.angular.z = float(max(min(w, 0.5), -0.5))

        self.get_logger().info(f"Dist: {self.filtered_depth:.2f}m | Pix-Err: {pixel_error:.1f}px | W: {drive_msg.angular.z:.3f}", throttle_duration_sec=0.5)

        self.cmd_vel_pub.publish(drive_msg)

        self.prev_lin_error = lin_error
        self.prev_pixel_error = pixel_error
        self.last_v = drive_msg.linear.x
        self.last_time = curr_time

    def perform_search(self):
        msg = Twist()
        msg.angular.z = self.search_angular_vel
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        try: self.cmd_vel_pub.publish(Twist())
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = ZedPersonFollower()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
