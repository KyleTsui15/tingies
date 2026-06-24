#Refined

# This is a high-level outline of the master node's logic and structure. The actual implementation will require careful handling of concurrency, error states, and edge cases, especially when coordinating multiple arms and ensuring safe operation in the shared build zone.

#MISC
    #PICKUP SEQUENCE
    #Move the arm down by "vertical_offset+0.05"
    #Wait until motion is complete.
    #Move the arm up by "vertical_offset"

    #DROP SEQUENCE (For building)
    # Move to arm down by "vertical_offset"
    # Spin servo id 10 CW 3 times to release the block by spinning lead screw
    # Wait 0.5 seconds.
    # Spin servo id 10 CCW 3 times to reset the gripper for the next pickup



# Master Node
    # Startup conditions
        # All arms at global_standby
    # vars --> 
        #Layer, 
        #Dictionary of block final locations, 
            #Layer --> Block type --> list of final locations --> state of that objective (unassigned, reserved, complete)
        #Global standby location
    # Inferred from Layer & Dictionary of block final locations --> current block type(s) needed published to /arm_{num}/block_class in a for loop for each arm. 
        # The object is marked as reserved but not removed from the dictionary until placement is complete to prevent multiple arms from trying to pick the same block.

#       ||
#       ||
#       \/

# Yolo_seg_node
    # Subscribes to /arm_{num}/block_class to know what block type to look for and publish the detected block position and angle to /arm_{num}/yolo_seg/object_pos and /arm_{num}/yolo_seg/object_angle

#       ||
#       ||
#       \/

# Vision_move_bridge node
    # Subscribes to /arm_{num}/yolo_seg/object_pos and /arm_{num}/yolo_seg/object_angle to get the detected block position and angle for arm_{num}
    # Possibly required to add another standby_pos here for multi-arm reseviors
    # Immediately moves to object without waiting for trigger 
        #Publishes to /arm_{num}/arm_state --> "picking"
    # Initiates PICKUP SEQUENCE
    # Moves to global_standby and waits for trigger to move to placement
        # Publishes to /arm_{num}/arm_state --> "standby"
    
# Master Node Step 2
    # Iterates through all /arm_{num}/arm_state and checks if any arm is currently in "placing"
    # If no arms it iterates through all /arm_{num}/arm_state and checks if any arm is currently in "standby"
    # If an arm is in "standby", it triggers the placement to final location
        #Publishes to /arm_{num}/arm_state --> "placing" before any movement is made
        #DROP SEQUENCE
        #Sets objective state to complete (removes from dictionary?)
        #Move back to global standby and sets arm state to "idle"