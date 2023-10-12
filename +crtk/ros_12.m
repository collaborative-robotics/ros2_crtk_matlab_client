classdef ros_12 < handle
    properties (Access = private)
        node;
    end

    methods (Static)
        function t = time(total_seconds)
            t = ros2time(total_seconds);
        end

        function msg = message(type)
            msg = ros2message(type);
        end
    end

    methods
        function self = ros_12(name)
            self.node = ros2node(name);
        end

        function delete(self)
            disp('deleting ros_12 object');
            delete(self.node);
        end

        function pub = publisher(self, topic, msg_type)
            pub = ros2publisher(self.node, topic, msg_type, Durability='transientLocal', Depth=10);
        end

        function sub = subscriber(self, topic, msg_type)
            sub = ros2subscriber(self.node, topic, msg_type);
        end

        function t = now(self)
            t = ros2time(self.node, "now");
        end

        function r = rate(self, desired_rate)
            r = ros2rate(self.node, desired_rate);
        end
    end
end