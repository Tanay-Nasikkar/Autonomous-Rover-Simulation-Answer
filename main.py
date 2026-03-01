from robot import Robot
from sensors.lidar import LidarScan
import matplotlib.pyplot as plt
import math
import csv

# -----------------------------------------------------
# Runtime modes & visualization toggles
# -----------------------------------------------------
MODE = "MANUAL"        # MANUAL | AUTO
SHOW_LIDAR = True
SHOW_ODOM = True


# -----------------------------------------------------
# Control commands (shared state)
# -----------------------------------------------------
v = 0.0   # linear velocity [m/s]
w = 0.0   # angular velocity [rad/s]


def on_key(event):
    """Keyboard control & visualization toggles."""
    global v, w, MODE, SHOW_LIDAR, SHOW_ODOM

    # --- visualization toggles ---
    if event.key == 'o':
        SHOW_ODOM = not SHOW_ODOM
        print(f"Odometry visualization: {'ON' if SHOW_ODOM else 'OFF'}")
        return

    if event.key == 'l':
        SHOW_LIDAR = not SHOW_LIDAR
        print(f"LiDAR visualization: {'ON' if SHOW_LIDAR else 'OFF'}")
        return

    # --- mode switching ---
    if event.key == 'm':
        MODE = "MANUAL"
        v = 0.0
        w = 0.0
        print("Switched to MANUAL mode")
        return

    if event.key == 'a':
        MODE = "AUTO"
        print("Switched to AUTO mode")
        return

    # --- manual control ---
    if MODE != "MANUAL":
        return

    if event.key == 'up':
        v += 1.5
    elif event.key == 'down':
        v -= 1.5
    elif event.key == 'left':
        w += 2.0
    elif event.key == 'right':
        w -= 2.0
    elif event.key == ' ':
        v = 0.0
        w = 0.0

    # clamp commands
    v = max(min(v, 6.0), -6.0)
    w = max(min(w, 6.0), -6.0)


if __name__ == "__main__":

    lidar = LidarScan(max_range=4.0)
    robot = Robot()

    plt.close('all')
    fig = plt.figure(num=2)
    fig.canvas.manager.set_window_title("Autonomy Debug View")
    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.show(block=False)

    dt = 0.01 

    """If required, this part of the code can be used inside the loop too, but I'm just using it here for efficiency"""
    # Load path
    path = []
    with open('path.csv', 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            path.append((float(row[0]), float(row[1])))

    # Pure Pursuit parameters
    lookahead_distance = 0.8
    target_v = 1.5
    current_path_index = 0

    # Obstacle Handling Strategy
    # "STOP" -> Part 3: Halts before impact
    # "AVOID" -> Part 4: Detours around obstacles
    OBSTACLE_STRATEGY = "AVOID"


    # -------------------------------------------------
    # Main simulation loop
    # -------------------------------------------------
    while plt.fignum_exists(fig.number):

        # ground truth pose
        real_x, real_y, real_theta = robot.get_ground_truth()
        # odometry estimate
        ideal_x, ideal_y, ideal_theta = robot.get_odometry()
        # LiDAR scan 
        lidar_ranges, lidar_points, lidar_rays, lidar_hits = lidar.get_scan((real_x, real_y, real_theta))

        if MODE == "AUTO":

            # ---------------------------------------------
            # write your autonomous code here!!!!!!!!!!!!!
            # ---------------------------------------------

            # Find the closest point to the rover to update path index
            min_dist = float('inf')
            closest_index = current_path_index
            
            # Search ahead locally to avoid searching the whole path each time
            search_range = min(len(path), current_path_index + 50)
            for i in range(current_path_index, search_range):
                px, py = path[i]
                dist = math.hypot(px - ideal_x, py - ideal_y)
                if dist < min_dist:
                    min_dist = dist
                    closest_index = i
            
            current_path_index = closest_index

            # Find the target point at the lookahead distance
            target_index = current_path_index
            for i in range(current_path_index, len(path)):
                px, py = path[i]
                dist = math.hypot(px - ideal_x, py - ideal_y)
                if dist >= lookahead_distance:
                    target_index = i
                    break
            
            # If we've reached the end of the path
            if target_index >= len(path) - 1 and math.hypot(path[-1][0] - ideal_x, path[-1][1] - ideal_y) < lookahead_distance:
                tx, ty = path[-1]
                # slow down near end
                dist_to_end = math.hypot(tx - ideal_x, ty - ideal_y)
                if dist_to_end < 0.2:
                    v = 0.0
                    w = 0.0
                else:
                    target_v_adjusted = max(0.5, dist_to_end)
                    v = target_v_adjusted
                    alpha = math.atan2(ty - ideal_y, tx - ideal_x) - ideal_theta
                    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
                    w = 2.0 * v * math.sin(alpha) / dist_to_end
            else:
                tx, ty = path[target_index]
                
                # Compute desired heading to target point
                alpha = math.atan2(ty - ideal_y, tx - ideal_x) - ideal_theta
                
                # Normalize angle to [-pi, pi]
                alpha = math.atan2(math.sin(alpha), math.cos(alpha))
                
                # Pure pursuit control law
                v = target_v
                # We limit the maximum steering angle
                w = 2.0 * v * math.sin(alpha) / lookahead_distance

            if OBSTACLE_STRATEGY == "STOP":
                # --- Part 3: Collision Prevention Mechanism ---
                # Define forward cone: +/- 30 degrees from the front
                # Lidar rays: 0 to 30 deg (indices 0,1,2,3), 330 to 350 deg (indices 33,34,35)
                forward_indices = [0, 1, 2, 3, 33, 34, 35]
                safety_threshold = 0.6  # Safety stopping distance in meters
                
                obstacle_detected = False
                for idx in forward_indices:
                    # lidar_ranges contains distance measurements for each beam
                    if lidar_ranges[idx] < safety_threshold:
                        obstacle_detected = True
                        break
                
                # If an obstacle is detected in the forward path, override commands to halt
                if obstacle_detected:
                    v = 0.0
                    w = 0.0

            elif OBSTACLE_STRATEGY == "AVOID":
                # --- Part 4: Detour With Dignity (Artificial Potential Field) ---
                repulsive_x = 0.0
                repulsive_y = 0.0
                
                # Repulsive field parameters
                safe_distance = 0.8     # Distance at which obstacles start repelling
                repulsion_gain = 1.2    # Strength of repulsion
                
                for i, dist in enumerate(lidar_ranges):
                    if dist < safe_distance:
                        # Angle of the beam relative to the robot's heading
                        angle_deg = i * 10
                        angle_rad = math.radians(angle_deg)
                        
                        # Compute repulsive force vector (pointing away from the obstacle)
                        # The force is stronger the closer the obstacle is
                        force = repulsion_gain * (1.0 / dist - 1.0 / safe_distance) * (1.0 / (dist ** 2))
                        
                        # Accumulate force in robot's local frame
                        repulsive_x -= force * math.cos(angle_rad)
                        repulsive_y -= force * math.sin(angle_rad)

                if abs(repulsive_x) > 0.01 or abs(repulsive_y) > 0.01:
                    # We are in an obstacle field, blend the pure pursuit with repulsion
                    
                    # Desired heading from Pure Pursuit in local frame is alpha
                    attractive_x = math.cos(alpha)
                    attractive_y = math.sin(alpha)
                    
                    # Resultant vector
                    result_x = attractive_x + repulsive_x
                    result_y = attractive_y + repulsive_y
                    
                    # New desired heading relative to the robot
                    result_alpha = math.atan2(result_y, result_x)
                    
                    # Control law based on blended heading
                    w = 2.0 * target_v * math.sin(result_alpha) / lookahead_distance
                    
                    # Reduce speed when navigating tight spaces (high repulsion)
                    repulsion_magnitude = math.hypot(repulsive_x, repulsive_y)
                    v = max(0.2, target_v - 0.2 * repulsion_magnitude)
                    
                    # Strict collision prevention as a final safety net
                    forward_indices = [0, 1, 2, 3, 33, 34, 35]
                    hard_stop_dist = 0.3
                    if any(lidar_ranges[idx] < hard_stop_dist for idx in forward_indices):
                        v = 0.0
                        w = 0.0

            # Clamp velocities
            v = max(min(v, 6.0), -6.0)
            w = max(min(w, 6.0), -6.0)


            # ---------------------------------------------
            # don't edit below this line (visualization & robot stepping)
            # ---------------------------------------------
        robot.step(
            lidar_points,
            lidar_rays,
            lidar_hits,
            v,
            w,
            dt,
            show_lidar=SHOW_LIDAR,
            show_odom=SHOW_ODOM
        )

        plt.pause(dt)
