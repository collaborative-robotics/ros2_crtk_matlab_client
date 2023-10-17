classdef utils < handle

    % Author(s): Anton Deguet
    %
    % Copyright (c) 2019-2022 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
    % Released under MIT License

    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
        ros_namespace; % namespace for this arm, should contain head/tail / (default is empty)
        class_instance;
        operating_state_instance;
    end

    % so derived class can extend this
    properties (Access = protected)
        % map of active publishers, used to stop all active publishers
        active_subscribers;
        % map of active subscribers, used to stop all active subscribers
        active_publishers;
        % ros messages instances to avoid runtime dynamic creation
        % these must be created in the constructor for crtk.utils
        std_msgs_Bool;
        std_msgs_StringStamped;
        sensor_msgs_JointState;
        geometry_msgs_Pose;
        geometry_msgs_Twist;
        geometry_msgs_Wrench;
    end

    % these should only be used by these class's methods
    properties (Access = private)
        ral;
        % operating state
        operating_state_timer;
        operating_state_subscriber;
        state_command_publisher;
        operating_state_data;
        operating_state_data_previous;
        % joint space
        measured_js_subscriber;
        setpoint_js_subscriber;
        servo_jp_publisher;
        servo_jr_publisher;
        servo_jf_publisher;
        move_jp_publisher;
        move_jr_publisher;
        % cartesian space
        measured_cp_subscriber;
        measured_cv_subscriber;
        measured_cf_subscriber;
        setpoint_cp_subscriber;
        setpoint_cv_subscriber;
        setpoint_cf_subscriber;
        servo_cp_publisher;
        servo_cf_publisher;
        move_cp_publisher;
    end


    methods(Static)

        function seconds = ros_time_to_secs(stamp)
            % Convert awkward rostime into a single double
            seconds = double(stamp.sec) + double(stamp.nanosec) * 10^-9;
        end

        function result = time_greater(t1, t2)
            if (t1.sec > t2.sec)
                result = true;
                return
            end
            if (t1.sec == t2.sec) && (t1.nanosec > t2.nanosec)
                result = true;
                return
            end
            result = false;
        end

        function frame = ros_pose_to_frame(pose)
            % convert ROS message type to homogeneous transforms
            position = trvec2tform([pose.position.x, pose.position.y, pose.position.z]);
            orientation = quat2tform([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]);
            frame = position * orientation;
        end

        function frame = ros_transform_to_frame(frame)
            % convert ROS message type to homogeneous transforms
            position = trvec2tform([frame.translation.x, frame.translation.y, frame.translation.z]);
            orientation = quat2tform([frame.rotation.w, frame.rotation.x, frame.rotation.y, frame.rotation.z]);
            frame = position * orientation;
        end

        function vector = ros_twist_to_vector(twist)
            % convert ROS message type to a single vector
            vector = [twist.linear.x,  twist.linear.y,  twist.linear.z, ...
                      twist.angular.x, twist.angular.y, twist.angular.z];
        end

        function vector = ros_wrench_to_vector(wrench)
            % convert ROS message type to a single vector
            vector = [wrench.force.x,  wrench.force.y,  wrench.force.z, ...
                      wrench.torque.x, wrench.torque.y, wrench.torque.z];
        end

        function pose = frame_to_ros_pose(frame, pose)
            % convert 4x4 homogeneous matrix to ROS transform
            pose.position.x = frame(1, 4);
            pose.position.y = frame(2, 4);
            pose.position.z = frame(3, 4);
            quaternion = tform2quat(frame);
            pose.orientation.w = quaternion(1);
            pose.orientation.x = quaternion(2);
            pose.orientation.y = quaternion(3);
            pose.orientation.z = quaternion(4);
        end

        function transform = frame_to_ros_transform(frame, transform)
            % convert 4x4 homogeneous matrix to ROS transform
            transform.translation.x = frame(1, 4);
            transform.translation.y = frame(2, 4);
            transform.translation.z = frame(3, 4);
            quaternion = tform2quat(frame);
            transform.rotation.w = quaternion(1);
            transform.rotation.x = quaternion(2);
            transform.rotation.y = quaternion(3);
            transform.rotation.z = quaternion(4);
        end

        function vector_to_ros_wrench(vector, wrench)
            % convert vector of 6 elements to ROS wrench
            wrench.force.x = vector(1);
            wrench.force.y = vector(2);
            wrench.force.z = vector(3);
            wrench.torque.x = vector(4);
            wrench.torque.y = vector(5);
            wrench.torque.z = vector(6);
        end

    end % methods(Static)

    methods

        function self = utils(class_instance, namespace, ral, operating_state_instance)
            narginchk(3, 4)
            if nargin == 4
                self.operating_state_instance = operating_state_instance;
            else
                self.operating_state_instance = class_instance;
            end

            self.active_publishers = containers.Map();
            self.active_subscribers = containers.Map();

            self.class_instance = class_instance;
            self.ros_namespace = namespace;
            self.ral = ral;
            self.operating_state_data = self.ral.message('crtk_msgs/OperatingState');
            % one time creation of messages to prevent lookup and creation at each call
            self.std_msgs_Bool = self.ral.message(rostype.std_msgs_Bool);
            self.std_msgs_StringStamped = self.ral.message('crtk_msgs/StringStamped');
            self.sensor_msgs_JointState = self.ral.message(rostype.sensor_msgs_JointState);
            self.geometry_msgs_Pose = self.ral.message(rostype.geometry_msgs_PoseStamped);
            self.geometry_msgs_Twist = self.ral.message(rostype.geometry_msgs_TwistStamped);
            self.geometry_msgs_Wrench = self.ral.message(rostype.geometry_msgs_WrenchStamped);
        end

        function delete(self)
            % delete all publishers that have been created
            for k = keys(self.active_publishers)
                publisher = self.active_publishers(k{1});
                delete(publisher);
            end
            % delete all subscribers that have been created
            for k = keys(self.active_subscribers)
                subscriber = self.active_subscribers(k{1});
                delete(subscriber);
            end
            % members always created
            delete(self.operating_state_subscriber); % delete subscriber first, its callback uses the timer below
            delete(self.operating_state_timer);
        end

        function full_topic = ros_topic(self, topic)
            % add trailing '/' to namespace if needed
            if strcmp(self.ros_namespace, '')
                full_topic = topic;
                return;
            end
            if strcmp(self.ros_namespace(end), '/')
                full_topic = strcat(self.ros_namespace, topic);
                return;
            end
            full_topic = strcat(self.ros_namespace, '/', topic);
        end

        function check_input_is_frame(self, frame)
            % verify that the input looks like a frame, i.e. homogeneous
            % matrix (4x4 matrix of real numbers).  This doesn't check if
            % the last row is [0 0 0 1].
            if ~isreal(frame)
                error('%s: input must be an array or real numbers, not %s', self.ros_namespace, class(frame));
            end
            if ~ismatrix(frame)
                error('%s: input must be a matrix', self.ros_namespace);
            end
            [nb_rows, nb_cols] = size(frame);
            if (nb_rows ~= 4) || (nb_cols ~=4)
                error('%s: input must be a 4x4 matrix, got %dx%d', self.ros_namespace, nb_rows, nb_cols);
            end
        end

        function operating_state_timeout(~, ~, ~) % first parameter is self, second is timer, third is this function
        end

        function operating_state_callback(self, ~, ~)
            self.operating_state_data_previous = self.operating_state_data;
            self.operating_state_data = self.operating_state_subscriber.LatestMessage;
            stop(self.operating_state_timer);
        end

        function [state] = operating_state(self)
            state = self.operating_state_subscriber.LatestMessage;
        end

        function [result] = wait_for_operating_state(self, expected_state, timeout)
            % wait for an operating state event and make sure the new
            % state is equal to the expected one
            time_left = timeout;
            tic;
            time_end = toc + timeout;
            while (isempty(self.operating_state_subscriber.LatestMessage) ...
                    || ~strcmp(self.operating_state_subscriber.LatestMessage.state, expected_state))...
                  && time_left > 0.0
                self.operating_state_timer.StartDelay = time_left;
                start(self.operating_state_timer);
                wait(self.operating_state_timer);
                % round is to avoid sub millisecond warning when setting
                % timer
                time_left = round(1000.0 * (time_end - toc)) / 1000.0;
            end
            % true if ended with some time left
            result = (time_left > 0.0);
        end

        function state_command(self, state)
            % send a state command, this method doesn't check if the
            % command is valid
            self.std_msgs_StringStamped.string = state;
            send(self.state_command_publisher, self.std_msgs_StringStamped);
        end

        function [result] = enable(self, timeout)
            % send the state command 'enable' and wait until the operating
            % state is 'ENABLED'
            if nargin == 1
                timeout = 0.0;
            end
            self.state_command('enable');
            if timeout ~= 0.0
                result = self.wait_for_operating_state('ENABLED', timeout);
            end
        end

        function [result] = disable(self, timeout)
            % send the state command 'disable' and wait until the operating
            % state is 'DISABLED'
            if nargin == 1
                timeout = 0.0;
            end
            self.state_command('disable');
            if timeout ~= 0.0
                result = self.wait_for_operating_state('DISABLED', timeout);
            end
        end

        function [result] = wait_for_homed(self, expected_home, timeout)
            % wait for an operating state event and make sure the member
            % IsHomed is equal to expected_home
            time_left = timeout;
            tic;
            time_end = toc + timeout;
            msg = @() self.operating_state_subscriber.LatestMessage;
            while (isempty(msg()) || msg().is_homed ~= expected_home) && time_left > 0.0
                self.operating_state_timer.StartDelay = time_left;
                start(self.operating_state_timer);
                wait(self.operating_state_timer);
                % round is to avoid sub millisecond warning when setting
                % timer
                time_left = round(1000.0 * (time_end - toc)) / 1000.0;
            end
            % true if ended with some time left
            result = (time_left > 0.0);
        end

        function [result] = home(self, timeout)
            % send the state command 'home' and wait until IsHomed is true
            if nargin == 1
                timeout = 0.0;
            end
            self.state_command('home');
            if timeout ~= 0.0
                result = self.wait_for_homed(true, timeout);
            end
        end

        function [busy] = is_busy(self, start_time)
            if nargin == 1
                start_time = crtk.ral.time(0.0);
            end
            busy = true;
            if crtk.utils.time_greater(self.operating_state_subscriber.LatestMessage.header.stamp, start_time)
                busy = self.operating_state_subscriber.LatestMessage.is_busy;
            end
        end

        function [result] = wait_for_busy(self, is_busy, start_time)
            % make sure event has not arrived yet
            msg = self.operating_state_subscriber.LatestMessage;
            secs = @(stamp) crtk.utils.ros_time_to_secs(stamp);
            if ~isempty(msg) && secs(msg.header.stamp) > secs(start_time) && (msg.is_busy == is_busy)
               result = true;
               return
            end
            % now wait for an operating state event
            while ~(self.operating_state_subscriber.LatestMessage.is_busy == is_busy)
                start(self.operating_state_timer);
                wait(self.operating_state_timer);
            end
            result = true;
        end

        function add_operating_state(self)
            % operating state subscriber
            self.operating_state_subscriber = ...
                self.ral.subscriber(self.ros_topic('operating_state'), 'crtk_msgs/OperatingState');
            self.operating_state_subscriber.NewMessageFcn = @self.operating_state_callback;
            self.active_subscribers('operating_state') = self.operating_state_subscriber;
            % accessors
            self.class_instance.addprop('operating_state');
            self.class_instance.operating_state = @self.operating_state;
            self.class_instance.addprop('is_busy');
            self.class_instance.is_busy = @self.is_busy;
            % operating state publisher
            cmd = 'state_command';
            self.state_command_publisher = ...
                self.ral.publisher(self.ros_topic(cmd), 'crtk_msgs/StringStamped');
            self.active_publishers(cmd) = self.state_command_publisher;
            self.class_instance.addprop(cmd);
            self.class_instance.state_command = @self.state_command;
            self.class_instance.addprop('enable');
            self.class_instance.enable = @self.enable;
            self.class_instance.addprop('disable');
            self.class_instance.disable = @self.disable;
            self.class_instance.addprop('home');
            self.class_instance.home = @self.home;

            % timer used for operating state
            self.operating_state_timer = timer('ExecutionMode', 'singleShot', ...
                                               'Name', strcat(self.ros_namespace, '_operating_state'), ...
                                               'ObjectVisibility', 'off', ...
                                               'TimerFcn', @self.operating_state_timeout);
            self.class_instance.addprop('wait_for_busy');
            self.class_instance.wait_for_busy = @self.wait_for_busy;
        end


        function [jp, jv, jf, timestamp] = measured_js(self)
            if isempty(self.measured_js_subscriber.LatestMessage)
                warning('measured_js has not received messages yet (topic %s)',...
                        self.measured_js_subscriber.TopicName);
                jp = [];
                jv = [];
                jf = [];
                timestamp = 0.0;
                return;
            end

            jp = self.measured_js_subscriber.LatestMessage.position;
            jv = self.measured_js_subscriber.LatestMessage.velocity;
            jf = self.measured_js_subscriber.LatestMessage.effort;
            timestamp = self.ros_time_to_secs(self.measured_js_subscriber.LatestMessage.header.stamp);
        end

        function add_measured_js(self)
            cmd = 'measured_js';
            self.measured_js_subscriber = ...
                self.ral.subscriber(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.class_instance.addprop(cmd);
            self.class_instance.measured_js = @self.measured_js;
            self.active_subscribers(cmd) = self.measured_js_subscriber;
        end


        function [jp, jv, jf, timestamp] = setpoint_js(self)
            if isempty(self.setpoint_js_subscriber.LatestMessage)
                warning('setpoint_js has not received messages yet (topic %s)',...
                        self.setpoint_js_subscriber.TopicName);
                jp = [];
                jv = [];
                jf = [];
                timestamp = 0.0;
                return;
            end
            jp = self.setpoint_js_subscriber.LatestMessage.position;
            jv = self.setpoint_js_subscriber.LatestMessage.velocity;
            jf = self.setpoint_js_subscriber.LatestMessage.effort;
            timestamp = self.ros_time_to_secs(self.setpoint_js_subscriber.LatestMessage.header.stamp);
        end

        function add_setpoint_js(self)
            cmd = 'setpoint_js';
            self.setpoint_js_subscriber = ...
                self.ral.subscriber(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.class_instance.addprop(cmd);
            self.class_instance.setpoint_js = @self.setpoint_js;
            self.active_subscribers(cmd) = self.setpoint_js_subscriber;
        end


        function servo_jp(self, jp)
            self.sensor_msgs_JointState.position = jp;
            send(self.servo_jp_publisher, self.sensor_msgs_JointState);
        end

        function add_servo_jp(self)
            cmd = 'servo_jp';
            self.servo_jp_publisher = ...
                self.ral.publisher(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.active_publishers(cmd) = self.servo_jp_publisher;
            self.class_instance.addprop(cmd);
            self.class_instance.servo_jp = @self.servo_jp;
        end


        function servo_jr(self, jp)
            self.sensor_msgs_JointState.position = jp;
            send(self.servo_jr_publisher, self.sensor_msgs_JointState);
        end

        function add_servo_jr(self)
            cmd = 'servo_jr';
            self.servo_jr_publisher = ...
                self.ral.publisher(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.active_publishers(cmd) = self.servo_jp_publisher;
            self.class_instance.addprop(cmd);
            self.class_instance.servo_jr = @self.servo_jr;
        end


        function servo_jf(self, jf)
            self.sensor_msgs_JointState.effort = jf;
            send(self.servo_jf_publisher, self.sensor_msgs_JointState);
        end

        function add_servo_jf(self)
            cmd = 'servo_jf';
            self.servo_jf_publisher = ...
                self.ral.publisher(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.active_publishers(cmd) = self.servo_jf_publisher;
            self.class_instance.addprop(cmd);
            self.class_instance.servo_jf = @self.servo_jf;
        end


        function [move_handle] = move_jp(self, jp)
            self.sensor_msgs_JointState.position = jp;
            move_handle = crtk.wait_move_handle(self.ral, self.operating_state_instance);
            send(self.move_jp_publisher, self.sensor_msgs_JointState);
        end

        function add_move_jp(self)
            cmd = 'move_jp';
            self.move_jp_publisher = ...
                self.ral.publisher(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.active_publishers(cmd) = self.move_jp_publisher;
            self.class_instance.addprop(cmd);
            self.class_instance.move_jp = @self.move_jp;
        end


        function [move_handle] = move_jr(self, jp)
            self.sensor_msgs_JointState.position = jp;
            move_handle = crtk.wait_move_handle(self.ral, self.operating_state_instance);
            send(self.move_jr_publisher, self.sensor_msgs_JointState);
        end

        function add_move_jr(self)
            cmd = 'move_jr';
            self.move_jr_publisher = ...
                self.ral.publisher(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.active_publishers(cmd) = self.move_jr_publisher;
            self.class_instance.addprop(cmd);
            self.class_instance.move_jr = @self.move_jr;
        end


        function [cp, timestamp] = measured_cp(self)
            if isempty(self.measured_cp_subscriber.LatestMessage)
                warning('measured_cp has not received messages yet (topic %s)',...
                        self.measured_cp_subscriber.TopicName);
                cp = [];
                timestamp = 0.0;
                return;
            end
            cp = self.ros_pose_to_frame(self.measured_cp_subscriber.LatestMessage.pose);
            timestamp = self.ros_time_to_secs(self.measured_cp_subscriber.LatestMessage.header.stamp);
        end

        function add_measured_cp(self)
            cmd = 'measured_cp';
            self.measured_cp_subscriber = ...
                self.ral.subscriber(self.ros_topic(cmd), rostype.geometry_msgs_PoseStamped);
            self.class_instance.addprop(cmd);
            self.class_instance.measured_cp = @self.measured_cp;
            self.active_subscribers(cmd) = self.measured_cp_subscriber;
        end


        function [cv, timestamp] = measured_cv(self)
            if isempty(self.measured_cv_subscriber.LatestMessage)
                warning('measured_cv has not received messages yet (topic %s)',...
                        self.measured_cv_subscriber.TopicName);
                cv = [];
                timestamp = 0.0;
                return;
            end
            cv = self.ros_twist_to_vector(self.measured_cv_subscriber.LatestMessage.twist);
            timestamp = self.ros_time_to_secs(self.measured_cv_subscriber.LatestMessage.header.stamp);
        end

        function add_measured_cv(self)
            cmd = 'measured_cv';
            self.measured_cv_subscriber = ...
                self.ral.subscriber(self.ros_topic(cmd), rostype.geometry_msgs_TwistStamped);
            self.class_instance.addprop(cmd);
            self.class_instance.measured_cv = @self.measured_cv;
            self.active_subscribers(cmd) = self.measured_cv_subscriber;
        end


        function [cf, timestamp] = measured_cf(self)
            if isempty(self.measured_cf_subscriber.LatestMessage)
                warning('measured_cf has not received messages yet (topic %s)',...
                        self.measured_cf_subscriber.TopicName);
                cf = [];
                timestamp = 0.0;
                return;
            end
            cf = self.ros_wrench_to_vector(self.measured_cf_subscriber.LatestMessage.wrench);
            timestamp = self.ros_time_to_secs(self.measured_cf_subscriber.LatestMessage.header.stamp);
        end

        function add_measured_cf(self)
            cmd = 'measured_cf';
            self.measured_cf_subscriber = ...
                self.ral.subscriber(self.ros_topic(cmd), rostype.geometry_msgs_WrenchStamped);
            self.class_instance.addprop('measured_cf');
            self.class_instance.measured_cf = @self.measured_cf;
            self.active_subscribers(cmd) = self.measured_cf_subscriber;
        end


        function [cp, timestamp] = setpoint_cp(self)
            if isempty(self.setpoint_cp_subscriber.LatestMessage)
                warning('setpoint_cp has not received messages yet (topic %s)',...
                        self.setpoint_cp_subscriber.TopicName);
                cp = [];
                timestamp = 0.0;
                return;
            end
            cp = self.ros_pose_to_frame(self.setpoint_cp_subscriber.LatestMessage.pose);
            timestamp = self.ros_time_to_secs(self.setpoint_cp_subscriber.LatestMessage.header.stamp);
        end

        function add_setpoint_cp(self)
            cmd = 'setpoint_cp';
            self.setpoint_cp_subscriber = ...
                self.ral.subscriber(self.ros_topic(cmd), rostype.geometry_msgs_PoseStamped);
            self.class_instance.addprop(cmd);
            self.class_instance.setpoint_cp = @self.setpoint_cp;
            self.active_subscribers(cmd) = self.setpoint_cp_subscriber;
        end


        function [cv, timestamp] = setpoint_cv(self)
            if isempty(self.setpoint_cv_subscriber.LatestMessage)
                warning('setpoint_cv has not received messages yet (topic %s)',...
                        self.setpoint_cv_subscriber.TopicName);
                cv = [];
                timestamp = 0.0;
                return;
            end
            cv = self.ros_twist_to_vector(self.setpoint_cf_subscriber.LatestMessage.twist);
            timestamp = self.ros_time_to_secs(self.setpoint_cv_subscriber.LatestMessage.header.stamp);
        end

        function add_setpoint_cv(self)
            cmd = 'setpoint_cv';
            self.setpoint_cv_subscriber = ...
                self.ral.subscriber(self.ros_topic(cmd), rostype.geometry_msgs_Twist);
            self.class_instance.addprop(cmd);
            self.class_instance.setpoint_cv = @self.setpoint_cv;
            self.active_subscribers(cmd) = self.setpoint_cv_subscriber;
        end


        function [cf, timestamp] = setpoint_cf(self)
            if isempty(self.setpoint_cf_subscriber.LatestMessage)
                warning('setpoint_cf has not received messages yet (topic %s)',...
                        self.setpoint_cf_subscriber.TopicName);
                cf = [];
                timestamp = 0.0;
                return;
            end
            cf = self.ros_wrench_to_vector(self.setpoint_cf_subscriber.LatestMessage.wrench);
            timestamp = self.ros_time_to_secs(self.setpoint_cf_subscriber.LatestMessage.header.stamp);
        end

        function add_setpoint_cf(self)
            cmd = 'setpoint_cf';
            self.setpoint_cf_subscriber = ...
                self.ral.subscriber(self.ros_topic(cmd), rostype.geometry_msgs_Wrench);
            self.class_instance.addprop(cmd);
            self.class_instance.setpoint_cf = @self.setpoint_cf;
            self.active_subscribers(cmd) = self.setpoint_cf_subscriber;
        end


        function servo_cp(self, cp)
            self.check_input_is_frame(cp);
            self.geometry_msgs_Pose.pose = self.frame_to_ros_pose(cp, self.geometry_msgs_Pose.pose);
            send(self.servo_cp_publisher, self.geometry_msgs_Pose);
        end

        function add_servo_cp(self)
            cmd = 'servo_cp';
            self.servo_cp_publisher = ...
                self.ral.publisher(self.ros_topic(cmd), rostype.geometry_msgs_PoseStamped);
            self.active_publishers(cmd) = self.servo_cp_publisher;
            self.class_instance.addprop(cmd);
            self.class_instance.servo_cp = @self.servo_cp;
        end


        function servo_cf(self, cf)
            self.vector_to_ros_wrench(cf, self.geometry_msgs_Wrench.wrench);
            send(self.servo_cf_publisher, self.geometry_msgs_Wrench);
        end

        function add_servo_cf(self)
            cmd = 'servo_cf';
            self.servo_cf_publisher = ...
                self.ral.publisher(self.ros_topic(cmd), rostype.geometry_msgs_WrenchStamped);
            self.active_publishers(cmd) = self.servo_cf_publisher;
            self.class_instance.addprop('servo_cf');
            self.class_instance.servo_cf = @self.servo_cf;
        end


        function [move_handle] = move_cp(self, cp)
            self.check_input_is_frame(cp);
            self.geometry_msgs_Pose.pose = self.frame_to_ros_pose(cp, self.geometry_msgs_Pose.pose);
            move_handle = crtk.wait_move_handle(self.ral, self.operating_state_instance);
            send(self.move_cp_publisher, self.geometry_msgs_Pose)
        end

        function add_move_cp(self)
            cmd = 'move_cp';
            self.move_cp_publisher = ...
                self.ral.publisher(self.ros_topic(cmd), rostype.geometry_msgs_PoseStamped);
            self.active_publishers(cmd) = self.move_cp_publisher;
            self.class_instance.addprop(cmd);
            self.class_instance.move_cp = @self.move_cp;
        end

    end % methods

end % class
