classdef wait_move_handle

    % Author(s): Anton Deguet
    %
    % Copyright (c) 2019-2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
    % Released under MIT License

    properties
        ral
        class_instance
        start_time
    end

    methods

        function self = wait_move_handle(ral, class_instance)
            self.ral = ral;
            self.class_instance = class_instance;
            self.start_time = ral.now();
        end

        function result = wait(self, is_busy)
            if nargin == 1
                is_busy = false;
            end
            result = self.class_instance.wait_for_busy(is_busy, self.start_time);
        end

        function result = is_busy(self, timeout)
            if nargin == 1
                timeout = 30.0;
            end
            now = self.ral.now();
            lapsed = ros2duration(now.sec - self.start_time.sec, now.nanosec - self.start_time.nanosec);
            if crtk.utils.ros_time_to_secs(lapsed) > timeout
               result = false;
            else
                result = self.class_instance.is_busy(self.start_time);
            end
        end

    end

end
