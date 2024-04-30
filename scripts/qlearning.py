#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
import numpy as np
from tf.transformations import euler_from_quaternion
import traceback
import pickle
import pandas as pd

# Define maximum distance and angle for object X and location Y detection
MAX_X_DISTANCE = 5
MAX_Y_DISTANCE = 5
NUM_X_BINS = 20
NUM_Y_BINS = 20

# Modify state_dim to reflect reduced state space
state_dim = (NUM_X_BINS, NUM_Y_BINS)

# Define discrete actions
NUM_ACTIONS = 7


# Global variables to store model states and laser scan data
model_states_msg = None
laser_scan_msg = None


class QLearningAgent:
    def __init__(self, state_dim, action_dim, epsilon=0.8, alpha=0.8, gamma=0.9):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.alpha = alpha  # Learning rate
        self.gamma = gamma  # Discount factor
        self.epsilon = epsilon  # Epsilon-greedy parameter
        self.Q = {}  # Initialize Q-table as an empty dictionary
        self.action_space = [i for i in range(action_dim)]  # Define action space
        self.visited_states = set()  # Set to store visited states
        self.state_visit_counts = {}  # Dictionary to store visit counts for each state

    def choose_action(self, state):
        rospy.loginfo("EPSILON")
        rospy.loginfo(self.epsilon)
        if np.random.rand() < self.epsilon or state not in self.Q:
            # Explore: return a random action
            action = np.random.choice(self.action_space)
            rospy.loginfo("Chose random action: %d", action)
            return action
        else:
            # Exploit: return action from Q-table
            action_values = self.Q[state]
            return np.argmax(action_values)

    def update_q_value(self, state, action, reward, next_state):
        # Get the Q-value for the current state or initialize it to 0
        q_value = self.Q.get(state, [0] * self.action_dim)
        # Update the Q-value corresponding to the chosen action
        q_value[action] += self.alpha * (reward + self.gamma * max(self.Q.get(next_state, [0] * self.action_dim)) - q_value[action])
        # Update the Q-table with the new Q-value for the current state
        self.Q[state] = q_value
        # Increment the visit count for the current state
        self.state_visit_counts[state] = self.state_visit_counts.get(state, 0) + 1

    def instantiate_q(self, episode, epsilon_decay_rate=0.1, **kwargs):
        rospy.loginfo("episode")
        rospy.loginfo(episode)

        if episode > 1 and 'Q_prev' in kwargs:
            self.Q = kwargs['Q_prev']
        else:
            rospy.loginfo("gets to else")
            for state in np.ndindex(self.state_dim):
                self.Q[state] = [0] * self.action_dim

        # Decay epsilon
        self.epsilon *= (1 - epsilon_decay_rate * (episode - 1))
        # Ensure epsilon is within bounds [0, 1]
        self.epsilon = max(0, min(self.epsilon, 1))

    def set_epsilon(self, epsilon):
        self.epsilon = epsilon

    def set_alpha(self, alpha):
        self.alpha = alpha

    def set_gamma(self, gamma):
        self.gamma = gamma


def calculate_state(robot_pose):
    x, y = robot_pose.position.x, robot_pose.position.y
    # Map continuous values to discrete bins for x and y coordinates
    x_bin = np.clip(int((x + MAX_X_DISTANCE) / (2 * MAX_X_DISTANCE) * NUM_X_BINS), 0, NUM_X_BINS - 1)
    y_bin = np.clip(int((y + MAX_Y_DISTANCE) / (2 * MAX_Y_DISTANCE) * NUM_Y_BINS), 0, NUM_Y_BINS - 1)
    return x_bin, y_bin


def calculate_reward(robot_pose, prev_state):
    # Calculate reward based on the robot's pose
    reward = 0
    
                 # Apply -1 reward for each new state
    if prev_state is not None:
        if state == prev_state:
            rospy.loginfo("State unchanged. No additional reward applied.")
        else:
            reward -= 1  # Apply -1 reward for each new state
            q_agent.update_q_value(state, action, reward, next_state)

            # Define reward locations in the discretized space
            reward_locations = {
                (9, 3): 250,   # Reward of +10 at (7, 7)
                (17, 17): 500  # Reward of +20 at (16, 16)
            }
            x_bin, y_bin = calculate_state(robot_pose)

            # Check if the robot's current position matches any of the reward locations
            if (x_bin, y_bin) in reward_locations:
                reward += reward_locations[(x_bin, y_bin)]

            # Check if the robot and the object are both at the target position
            if (x_bin, y_bin) == (9, 10):
                # Add reward for reaching the target position
                reward += 15

    else:
        reward -= 1  # Apply -1 reward for the first state
        q_agent.update_q_value(state, action, reward, next_state)

    return reward


def differentiate_objects(laser_data, angles, distance_threshold=1, width_threshold=1, height_threshold=0.5):
    objects = {}

    # Iterate through laser data
    for i, (distance, angle) in enumerate(zip(laser_data, angles)):
        if distance < distance_threshold: # Threshold distance for considering an object
            # Calculate the width and height of the object
            object_width, object_height, object_length = calculate_object_dimensions(laser_data, angles, i, distance_threshold)
            
            # Determine the type of the object based on its dimensions
            if object_width < width_threshold and object_height < height_threshold:
                object_type = 'small_cube'
            elif object_width > width_threshold and object_height > height_threshold:
                object_type = 'large_rectangle'
            else:
                object_type = 'other'

            # Determine the side of the object relative to the robot
            if angle < 0:
                side = 'right'
            else:
                side = 'left'
            
            objects['object_{}_{}'.format(i, side)] = {'type': object_type, 'distance': distance, 'width': object_width, 'height': object_height} 

    return objects



def calculate_object_dimensions(laser_data, angles, index, distance_threshold):
    # Find the range of laser data corresponding to the object at the given index
    object_range = get_object_range(laser_data, index, distance_threshold)
    if object_range:
        # Calculate the width, height, and length of the object
        object_width = max(object_range) - min(object_range)
        object_height = len(object_range)  # Height as the number of laser measurements
        object_length = calculate_object_length(object_range, angles)
        return object_width, object_height, object_length
    else:
        return 0, 0, 0  # Return 0 for width, height, and length if object range is empty or not 
    
def calculate_object_length(object_range, angles):
    # Calculate the length of the object using laser data and angles
    object_start_angle = angles[0]
    object_end_angle = angles[-1]
    object_length = abs(object_end_angle - object_start_angle)
    return object_length

def get_object_range(laser_data, index, distance_threshold):
    # Find the range of laser data corresponding to the object at the given index
    object_range = []
    current_index = index
    while current_index < len(laser_data) and laser_data[current_index] < distance_threshold:
        object_range.append(laser_data[current_index])
        current_index += 1
    return object_range

def reset_objects():
    original_positions = {
        'week7bot': (7.531148, -7.437854, 0.1),
    }

    # Create a proxy to the '/gazebo/set_model_state' service
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Reset the positions of all objects
    for model_name, original_pose in original_positions.items():
        try:
            # Create a ModelState message
            model_state_msg = ModelState()
            model_state_msg.model_name = model_name
            model_state_msg.pose.position.x = original_pose[0]
            model_state_msg.pose.position.y = original_pose[1]
            model_state_msg.pose.position.z = original_pose[2]

            # Call the service to set the model     
            response = set_model_state(model_state_msg)
            
            rospy.loginfo("Resetting position of {} to ({}, {}, {})".format(
                model_name, original_pose[0], original_pose[1], original_pose[2]))
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))


def model_states_callback(msg):
    global model_states_msg
    model_states_msg = msg


def laser_scan_callback(msg):
    global laser_scan_msg
    laser_scan_msg = msg


def get_laser_scan_data(laser_msg):
    # Extract laser data from the laser message
    ranges = laser_msg.ranges  # List of distances to obstacles
    angle_min = laser_msg.angle_min  # Minimum angle of the laser scan
    angle_max = laser_msg.angle_max  # Maximum angle of the laser scan
    angle_increment = laser_msg.angle_increment  # Angular distance between measurements
    # Calculate the angles for each laser scan measurement
    angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)
    return ranges, angles


def perform_action(action):
    control_msg = Twist()
    angular_velocity_scale = 0.7  # Scale factor for angular velocity
    if action > 1:  # If action is forward motion
        if action == 2:  # Forward motion with reduced linear velocity
            control_msg.linear.x = 4.0  # Reduced forward 
            control_pub.publish(control_msg)

            rospy.sleep(0.5)

        elif action == 3:  # Forward motion with default linear velocity
            control_msg.linear.x = 8.0  # Default forward
            control_pub.publish(control_msg)

            rospy.sleep(1)


        elif action == 4:  # Forward motion with default linear velocity
            control_msg.linear.x = 10.0  # Default forward velocity
            control_pub.publish(control_msg)
            rospy.sleep(1.5)

        elif action == 5:  # Forward motion with default linear velocity
            control_msg.linear.x = 12.0  # Default forward velocity
            control_pub.publish(control_msg)
            rospy.sleep(2)

        else:
            rospy.logwarn("Invalid action: %d", action)
    else:  # If action is angular motion
        if action == 0:  # Turn left 90 degrees
            angle = np.pi / 2  # 90 degrees in radians
            control_msg.angular.z = -angle * angular_velocity_scale  # Set angular velocity for turning left
            control_pub.publish(control_msg)
            rospy.loginfo("Turn Left Normal")
            rospy.sleep(1.5)
        elif action == 1:  # Turn right 90 degrees
            angle = np.pi / 2  # 90 degrees in radians
            control_msg.angular.z = angle * angular_velocity_scale  # Set angular velocity for turning right
            control_pub.publish(control_msg)
            rospy.loginfo("Turn right Normal")
            rospy.sleep(1.5)
        else:
            rospy.logwarn("Invalid action: %d", action)
    
    return control_msg


if __name__ == '__main__':
    try:
        rospy.init_node('q_learning_control')
        state_dim = (NUM_X_BINS, NUM_Y_BINS)
        action_dim = NUM_ACTIONS
        control_pub = rospy.Publisher('week7bot/cmd_vel', Twist, queue_size=1)
        sub_model_states = rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
        sub_laser_scan = rospy.Subscriber('/week7bot/laser/scan', LaserScan, laser_scan_callback)

        episode_count = 1
        total_episodes = 10
        rate = rospy.Rate(20)
        prev_robot_pose = Pose()  # Initialize prev_robot_pose with a default Pose
        prev_state = None  # Initialize prev_state
        q_agent = QLearningAgent(state_dim, 8)

        while not rospy.is_shutdown() and episode_count <= total_episodes:  # Change 10 to desired number of episodes
            DONE = False
            total_reward = 0  # Initialize total reward for the episode
            rate.sleep()

            try:
                if episode_count == 1:
                    q_agent.instantiate_q(episode_count)
                else:
                    q_agent_prev = q_agent
                    q_agent.instantiate_q(episode_count, Q_prev=q_agent_prev.Q)

                while not DONE:
                    if laser_scan_msg is None or model_states_msg is None:
                        continue  # Skip iteration if messages are not received yet

                    laser_data, angles = get_laser_scan_data(laser_scan_msg)  # Get laser scan data
                    detected_objects = differentiate_objects(laser_data, angles)

                    avoid_wall = False  # Flag to indicate if the robot needs to avoid a wall
                    for obj_key, obj_data in detected_objects.items():
                        rospy.loginfo(obj_data['width'])

                        if obj_data['width'] > 0.1:
                            # rospy.loginfo(obj_key)
                            avoid_wall = True
                            break  # one wide wall is enough to trigger the action

                    robot_pose = model_states_msg.pose[model_states_msg.name.index('week7bot')]
                    movable_object_pose = model_states_msg.pose[model_states_msg.name.index('movable_box')]

                    # If a wide obstacle is detected, immediately publish a turn command
                    if avoid_wall:
                        if any('object_' in obj_key for obj_key in detected_objects.keys()):
                            for obj_key, obj_data in detected_objects.items():
                                rospy.loginfo
                                if 'object_' in obj_key and obj_data['distance'] < 1 and obj_data['width'] > 0.2:
                                    rospy.loginfo("Obstacle detected")
                                    if 'right' in obj_key:
                                        rospy.loginfo("Obstacle detected on the right side. Turning 90 degrees to the left.")
                                        # rospy.loginfo(obj_key)
                                        # rospy.loginfo(obj_data)
                                        if obj_data['width'] > 0.8:
                                            control_msg = Twist()
                                            control_msg.linear.x = 0.0  # No forward velocity
                                            control_msg.angular.z = 0.3  # Turn 90 degrees to the left
                                            control_pub.publish(control_msg)
                                            rospy.sleep(1)
                                        else:

                                            control_msg = Twist()
                                            control_msg.linear.x = 0.0  # No forward velocity
                                            control_msg.angular.z = 0.7  # Turn 90 degrees to the left
                                            control_pub.publish(control_msg)
                                            rospy.sleep(1.5)
                                        break  # Exit the loop after publishing the turn command
                                    elif 'left' in obj_key: 
                                        rospy.loginfo("Obstacle detected on the left side. Turning 90 degrees to the right.")
                                        # rospy.loginfo(obj_key)
                                        # rospy.loginfo(obj_data)
                                        if obj_data['width'] > 0.8:
                                            control_msg = Twist()
                                            control_msg.linear.x = 0.0  # No forward velocity
                                            control_msg.angular.z = -0.3  # Turn 90 degrees to the left
                                            control_pub.publish(control_msg)
                                            rospy.sleep(1)
                                        else:
                                            control_msg = Twist()
                                            control_msg.linear.x = 0.0  # No forward velocity
                                            control_msg.angular.z = -0.7  # Turn 90 degrees to the right
                                            control_pub.publish(control_msg)
                                            rospy.sleep(1.5)
                                        break  # Exit the loop after publishing the turn command
                        continue  # Skip the normal action selection process
                    else:

                        state = calculate_state(robot_pose)
                        action = q_agent.choose_action(state)
                        control_msg = perform_action(action)

                    next_robot_pose = model_states_msg.pose[model_states_msg.name.index('week7bot')]
                    next_state = calculate_state(next_robot_pose)
                    reward = calculate_reward(robot_pose, prev_state)
                    prev_robot_pose = robot_pose  # Update previous robot pose
                    q_agent.update_q_value(state, action, reward, next_state)
                    total_reward += reward  # Update total reward for the episode
                    rospy.loginfo("REWARD")
                    rospy.loginfo(reward)
                    rospy.loginfo("Total")
                    rospy.loginfo(total_reward)
                    rospy.loginfo("EP")
                    rospy.loginfo(episode_count)

                    prev_state = state  # Update previous state for the next iteration
                    # rospy.loginfo("STATE")
                    # rospy.loginfo(state)

                    if state == (0, 19):
                        # Reset the environment
                        reset_objects()
                        # Pause for 2 seconds
                        rospy.loginfo("Resetting environment. Pausing for 2 seconds before starting the next episode...")
                        rospy.sleep(4)
                        # Increment the episode count
                        episode_count += 1
                        # Reset the prev_state
                        prev_state = None
                        DONE = True

                        if episode_count == total_episodes:
                            rospy.loginfo("EP/TOTAL")
                            rospy.loginfo(episode_count)
                            rospy.loginfo(total_episodes)

                            with open('final_q_table-test1.pkl', 'wb') as f:
                                pickle.dump(q_agent.Q, f)
                            rospy.loginfo("Final Q-table saved")
                            with open('final_q_table-test1.pkl', 'rb') as f:
                                q_table_data = pickle.load(f)
                            column_names = ['Action_{}'.format(i) for i in range(NUM_ACTIONS)]
                            q_table_df = pd.DataFrame.from_dict(q_table_data, orient='index', columns=column_names)

                            # Display the Q-table DataFrame
                            print("Q-table:")
                            print(q_table_df)
                        break
                    
        
                rospy.loginfo("Total reward for episode %d: %d", episode_count - 1, total_reward)

            except rospy.ROSInterruptException:
                rospy.loginfo("ROS interrupt detected.")
            except Exception as e:
                rospy.logerr("An error occurred: %s", str(e))
                rospy.logerr(traceback.format_exc())  # Print the stack trace
        # Load the Q-table from file



                
    except Exception as e:
        rospy.logerr("An error occurred in the main block: %s", str(e))
        rospy.logerr(traceback.format_exc())  # Print the stack trace
