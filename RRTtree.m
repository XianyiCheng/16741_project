classdef RRTtree < matlab.mixin.Copyable
    
    properties
        vertex;
        counter = 2;% cur number of nodes + 1
    end

    methods
        function obj = RRTtree(varargin)
            % init tree
            % X_start [x, y, theta]
            % start_contacts 4xn, [x, y, nx, ny]
            % start_finger 
            assert nargin == 3;
            Xstart = varargin{1};
            start_contacts = varargin{2};
            start_finger = varargin{3};
            vertex(1).x = Xstart(1);
            vertex(1).y = Xstart(2);
            vertex(1).theta = Xstart(3);
            vertex(1).env_contacts = start_contacts; 
            vertex(1).finger_contacts = start_finger; 
            vertex(1).parent = -1;
        end

        function obj = add_node(obj, Xnear_ind, Xnew, Xnew_env_contacts, Xnew_finger_contacts);
            obj.vertex(obj.counter).x = Xstart(1);
            obj.vertex(T.counter).y = Xstart(2);
            obj.vertex(T.counter).theta = Xstart(3);
            obj.vertex(T.counter).env_contacts = Xnew_env_contacts; 
            obj.vertex(T.counter).finger_contacts = Xnew_finger_contacts; 
            obj.vertex(T.counter).parent = Xnear_ind;
            obj.counter += 1;
        end
        
        function d = treeDist(X1, X2)
            % comparing both loc and angle
            % assume size(X1) == size(X2)
            % d = pdist([X1;X2],'euclidean')
            
            
            % comparing angles only
            % only consider theta when measuring distance and comparing to goal
            d = pdist([X1(3);X2], 'euclidean');
        end

        function [closest_ind, cur_dist] = nearestNeighbor(obj, Xgoal)
            cur_dist = inf;
            closest_ind = -1;
            for i = 1:obj.counter-1
                cur = obj.vertex(i); 
                state = [cur.x, cur.y, cur.theta];
                this_dist = treeDist(state, Xgoal)
                if this_dist < cur_dist
                    cur_dist = this_dist;
                    closest_ind = i;
                end
            end
        end

        function get_path(obj, end_ind)
            % display one path of all states
            cur = end_ind
            while cur ~= -1
                disp('obj state is \n')    
                [obj.vertex(cur).x, obj.vertex(cur).y, obj.vertex(cur).theta]
                disp('env contacts are \n') 
                obj.vertex(cur).env_contacts;
                disp('finger contacts are \n') 
                obj.vertex(cur).finger_contacts; 
                disp('-------NEXT-------')
                cur = obj.vertex(cur).parent;
            end
        end

    end

end