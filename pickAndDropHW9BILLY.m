function main_hw9(object_number_arg)    
    % The goal is to pick up and item on the desk, and then drop it into the
    % green bin on the floor
    % make sure to initiate ROS, connect to gazebo
    
    % manual inputs
    object_number = object_number_arg;
    % pick up coordinates
    gCan1_number = 20;
    gCan2_number = 21;
    gCan3_number = 22;
    gCan4_number = 23;
    rCan1_number = 24;
    rCan2_number = 25;
    rCan3_number = 26;
    yCan1_number = 27;
    yCan2_number = 28;
    yCan3_number = 29;
    yCan4_number = 30;
    rBottle1_number = 31;
    rBottle2_number = 32;
    bBottle1_number = 33;
    bBottle2_number = 34;
    bBottle3_number = 35;
    yBottle1_number = 36;
    yBottle2_number = 37;
    yBottle3_number = 38;
    yBottle4_number = 39;
    % rCan3_coord = [-0.04 0.8 0.19]; % (for reference)
    
    % getting pose for object (approved)
    models = getModels;      % Extract gazebo model list
    model_name = models.ModelNames{object_number}; 
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);
    pause(1)
    
    % q_ready: elbow bent forward 90 deg (approved)
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                               'control_msgs/FollowJointTrajectory', ...
                                               'DataFormat', 'struct');
    traj_goal = rosmessage(pick_traj_act_client); 
    qr = [atan2(-mat_R_T_M(1, 4), mat_R_T_M(2, 4)) 0 pi/2 -pi/2 0 0];
    traj_goal = convert2ROSPointVec(qr,ros_cur_jnt_state_msg.Name,1,1,traj_goal);
    disp('Sending q_ready goal...');
    pause(1)
    sendGoalAndWait(pick_traj_act_client,traj_goal);
    pause(1)
    disp('q_ready goal complete.');
    pause(1)
    
    % travel down to object (approved)
    ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.1;   % Vertical offset for top-down approach
    ops("traj_duration")       = 2;     % Traj duration (secs)   
    grip_result                = -1;           % Init to failure number  
    fprintf('Traveling to model: %s \n',model_name);
    pause(1)
    strategy = 'topdown';
    over_R_T_M = lift(mat_R_T_M, 0.1);
    traj_result = moveTo(over_R_T_M,ops);
    if object_number >= 31 && object_number <= 39
        mat_R_T_M(3,4) = 0.24;
    elseif object_number == 28
        mat_R_T_M(3,4) = 0.13;
        mat_R_T_M = mat_R_T_M*trotz(pi/8);
        pause(1)
    end
    traj_result = moveTo(mat_R_T_M,ops);
    fprintf('Traveling to model completed: %s \n',model_name);
    pause(2)
    
    % determining gripper close value
    if object_number >= 31 && object_number <= 39
        % close gripper on bottle
        grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                                  'control_msgs/FollowJointTrajectory',...
                                                  'DataFormat','struct');
        grip_msg = rosmessage(grip_action_client);
        gripPos = 0.515; 
        grip_goal = packGripGoal_struct(gripPos,grip_msg);
        disp('Sending grip goal...');
        pause(1)
        sendGoalAndWait(grip_action_client,grip_goal);
        pause(1)
        disp('Grip goal complete.');
        pause(2)
    elseif object_number == 28
        % close gripper on ycan2 (horizontal) (approved)
        grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                                  'control_msgs/FollowJointTrajectory',...
                                                  'DataFormat','struct');
        grip_msg = rosmessage(grip_action_client);
        gripPos = 0.24; 
        grip_goal = packGripGoal_struct(gripPos,grip_msg);
        disp('Sending grip goal...');
        pause(1)
        sendGoalAndWait(grip_action_client,grip_goal);
        pause(1)
        disp('Grip goal complete.');
        pause(2)
    else
        % close gripper on can (approved)
        grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                                  'control_msgs/FollowJointTrajectory',...
                                                  'DataFormat','struct');
        grip_msg = rosmessage(grip_action_client);
        gripPos = 0.23; 
        grip_goal = packGripGoal_struct(gripPos,grip_msg);
        disp('Sending grip goal...');
        pause(1)
        sendGoalAndWait(grip_action_client,grip_goal);
        pause(1)
        disp('Grip goal complete.');
        pause(2)
    end
    
    % determining which bin to go to (approved)
    if object_number >= 31 && object_number <= 39
        % travelling to blue bin drop location
        disp('Attempting place...')
        blueBin = [-0.45, 0.4, 0.25, -pi/2, -pi 0];
        place_pose = set_manual_goal(blueBin);
        strategy = 'topdown';
        fprintf('Moving to blue bin...');
        ret = moveToBin(strategy,mat_R_T_M,place_pose);
    elseif object_number == 28
        % travelling to green bin drop location
        disp('Attempting place...')
        greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
        place_pose = set_manual_goal(greenBin);
        strategy = 'topdown';
        fprintf('Moving to green bin...');
        ret = moveToBin(strategy,mat_R_T_M,place_pose);
    else
        % travelling to green bin drop location
        disp('Attempting place...')
        greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
        place_pose = set_manual_goal(greenBin);
        strategy = 'topdown';
        fprintf('Moving to green bin...');
        ret = moveToBin(strategy,mat_R_T_M,place_pose);
    end
    pause(2)
    
    % q_home (approved)
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                               'control_msgs/FollowJointTrajectory', ...
                                               'DataFormat', 'struct');
    traj_goal = rosmessage(pick_traj_act_client); 
    qr = [0 0 0 0 0 0];
    traj_goal = convert2ROSPointVec(qr,ros_cur_jnt_state_msg.Name,1,1,traj_goal);
    disp('Sending q_home goal...');
    pause(1)
    sendGoalAndWait(pick_traj_act_client,traj_goal);
    pause(1)
    disp('q_home goal complete.');
    pause(1)