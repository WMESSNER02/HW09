function matlab_pose = ros2matlabPose(p)
    %----------------------------------------------------------------------
    % ros2matlabPose
    % Converts ros pose to matlab pose according to type. Handles:
    %
    % gazebo_msgs/GetModelStateResponse:
    % - Position.XYZ 
    % - Orientation.XYZW
    % geometry_msgs/TransformStamped
    % - Transform.Translation.XYZ
    % - Transform.Rotation.XYZW
    % 
    % Inputs:
    % p (ROS): pose
    %
    % Outputs: 
    % matlab_pose (Transform): 4x4 matrix
    %
    % Extracts position into p and orientation into a quaternion iva the RVC 
    % Toolbox to instantiate UnitQuaternion and build a homogeneous
    % transform by composing orientation and translation.
    %----------------------------------------------------------------------
    
    % geometry_msgs/Pose
    if strcmp(p.MessageType, 'gazebo_msgs/GetModelStateResponse')
        pos = [p.Pose.Position.X, ...
               p.Pose.Position.Y, ...
               p.Pose.Position.Z];

        q = UnitQuaternion(p.Pose.Orientation.W, ...
                           [p.Pose.Orientation.X, ...
                            p.Pose.Orientation.Y, ...
                            p.Pose.Orientation.Z]);
            
    % 'geometry_msgs/TransformStamped'
    elseif strcmp(p.MessageType, 'geometry_msgs/TransformStamped')
        pos = [p.Transform.Translation.X, ...
               p.Transform.Translation.Y, ...
               p.Transform.Translation.Z];

        q = UnitQuaternion(p.Transform.Rotation.W, ...
                           [p.Transform.Rotation.X, ...
                            p.Transform.Rotation.Y, ...
                            p.Transform.Rotation.Z]);
    end

    % Build matlab pose
    matlab_pose = transl(pos) * q.T;
end