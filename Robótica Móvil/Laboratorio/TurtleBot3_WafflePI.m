classdef TurtleBot3_WafflePI < handle
    properties (Access = private)
        bot_n = [];

        % Camera
        camSub = [];

        % Lidar
        lidarSub = [];
        d_min = 0.01;
        d_max = 3.0;

        % Odometry
        odomSub = [];
        resetPub = [];
        resetMsg = [];

        % IMU
        imuSub = [];
        magneticFieldSub = [];

        % Battery State
        batterySub = [];

        % Velocity Command
        velPub = [];
        velMsg = [];
    end
    
    methods
        function obj = TurtleBot3_WafflePI(bot_ip)
            try
                disp('Connecting to TurtleBot3 Waffle PI...')
                n = str2double(bot_ip(20:21));
                obj.bot_n = num2str(n);

                rosshutdown
                rosinit(bot_ip)

                % Camera
                obj.camSub = rossubscriber(['/bot' obj.bot_n '/raspicam_node/image/compressed'],'sensor_msgs/CompressedImage');
                
                % Lidar
                obj.lidarSub = rossubscriber(['/bot' obj.bot_n '/scan'],'sensor_msgs/LaserScan');
                
                % Odometry
                obj.odomSub = rossubscriber(['/bot' obj.bot_n '/odom'],'nav_msgs/Odometry');
                obj.resetPub = rospublisher(['/bot' obj.bot_n '/reset'],'std_msgs/Empty');
                obj.resetMsg = rosmessage(obj.resetPub);

                % Velocity Command
                obj.velPub = rospublisher(['/bot' obj.bot_n '/cmd_vel'],"geometry_msgs/Twist") ;
                obj.velMsg = rosmessage(obj.velPub);
                
                % Battery State
                obj.batterySub = rossubscriber(['/bot' obj.bot_n '/battery_state'],'sensor_msgs/BatteryState');

                % IMU
                obj.imuSub = rossubscriber(['/bot' obj.bot_n '/imu'],'sensor_msgs/Imu');
                obj.magneticFieldSub = rossubscriber(['/bot' obj.bot_n '/magnetic_field'],'sensor_msgs/MagneticField');

                disp(' done.')
            catch E
                error(E)
            end
        end

        %% Camera
        function img = Get_Image (obj)
            try 
                camMsg = receive(obj.camSub);
                camMsg.Format = 'bgr8; jpeg compressed bgr8';
            
                img = readImage(camMsg);
            catch E
                error(E)
            end
        end

        %% Lidar
        function [Angles,Ranges] = Get_LaserScans (obj)
            try
                scanMsg = receive(obj.lidarSub);

                Angles = deg2rad(1:360)';
                Ranges = scanMsg.Ranges;

                I = obj.d_min<=Ranges & Ranges<=obj.d_max;

                Angles = Angles(I);
                Ranges = Ranges(I);
            catch E
                error(E)
            end
        end

        %% Baterry
        function [V, I] = Get_BatteryState (obj)
            try
                scanMsg = receive(obj.batterySub);

                V = double(scanMsg.Voltage);
                I = double(scanMsg.Current);
            catch E
                error(E)
            end
        end

        %% Odometry
        function Reset_Odometry (obj)
            try
                disp('Restarting odometry...')
                send(obj.resetPub,obj.resetMsg)
                pause(0.5)

                disp(' done.')
            catch E
                error(E)
            end
        end

        function p = Get_Pose (obj)
            try
                odomMsg = receive(obj.odomSub);
                pose = odomMsg.Pose.Pose;
                x = pose.Position.X;
                y = pose.Position.Y;
                
                quat = pose.Orientation;
                angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
                theta = angles(1);  
                
                p = [x y theta]';
            catch E
                error(E)
            end
        end

        %% IMU
        function [g,a,m] = Get_ImuData (obj)
            try
                scanMsg = receive(obj.imuSub);
                a = [scanMsg.LinearAcceleration.X scanMsg.LinearAcceleration.Y scanMsg.LinearAcceleration.Z]';
                g = [scanMsg.AngularVelocity.X scanMsg.AngularVelocity.Y scanMsg.AngularVelocity.Z]';

                scanMsg = receive(obj.magneticFieldSub);
                m = [scanMsg.MagneticField_.X scanMsg.MagneticField_.Y scanMsg.MagneticField_.Z]';
            catch E
                error(E)
            end
        end

        function q = Get_Quaternion (obj)
            try
                scanMsg = receive(obj.imuSub);
                q = [scanMsg.Orientation.W scanMsg.Orientation.X scanMsg.Orientation.Y scanMsg.Orientation.Z]';

            catch E
                error(E)
            end
        end

        %% Velocity Command
        function Set_Velocity (obj,v,w)
            try
                v = max(v,-0.2); v = min(v,0.2);
                w = max(w,-0.9); w = min(w,0.9);

                obj.velMsg.Linear.X = v;
                obj.velMsg.Angular.Z = w;

                send(obj.velPub,obj.velMsg)
            catch E
                error(E)
            end
        end
    end

    methods(Static)
        function Stop_Connection ()
            disp('Stopping connection...')
            rosshutdown
            disp(' done.')
        end
    end
end