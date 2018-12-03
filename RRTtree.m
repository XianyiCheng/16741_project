classdef RRTtree < matlab.mixin.Copyable
    
    properties
        vertex;
        counter = 2;% cur number of nodes + 1
    end

    methods
        function obj = RRTtree(Xstart, start_contacts, start_finger)
            % init tree
            % X_start [x, y, theta]
            % start_contacts 4xn, [x, y, nx, ny]
            % start_finger 
            obj.vertex(1).x = Xstart(1);
            obj.vertex(1).y = Xstart(2);
            obj.vertex(1).theta = Xstart(3);
            obj.vertex(1).env_contacts = start_contacts; 
            obj.vertex(1).finger_contacts = start_finger; 
            obj.vertex(1).parent = -1;
        end

        function obj = add_node(obj, Xnear_ind, Xnew, Xnew_env_contacts, Xnew_finger_contacts)
            obj.vertex(obj.counter).x = Xnew(1);
            obj.vertex(obj.counter).y = Xnew(2);
            obj.vertex(obj.counter).theta = Xnew(3);
            obj.vertex(obj.counter).env_contacts = Xnew_env_contacts; 
            obj.vertex(obj.counter).finger_contacts = Xnew_finger_contacts; 
            obj.vertex(obj.counter).parent = Xnear_ind;
            obj.counter = obj.counter+ 1;
        end
        
        function d = treeDist(obj, X1, X2)
            if numel(X2) == 1
                % comparing both loc and angle
                % assume size(X1) == size(X2)
                %d = pdist([X1;X2],'euclidean');
                d = norm(X1-X2);
            elseif numel(X2) == 3
                % comparing angles only
                % only consider theta when measuring distance and comparing to goal
                %d = pdist([X1(3);X2], 'euclidean');
                d = norm(X1(3)-X2(3));
            end
        end

        function [closest_ind, cur_dist] = nearestNeighbor(obj, Xgoal)
            cur_dist = inf;
            closest_ind = -1;
            for i = 1:obj.counter-1
                cur = obj.vertex(i); 
                state = [cur.x, cur.y, cur.theta];
                this_dist = obj.treeDist(state, Xgoal);
                if this_dist < cur_dist
                    cur_dist = this_dist;
                    closest_ind = i;
                end
            end
        end

        function show_ind = get_path(obj, end_ind)
            % display one path of all states
            cur = end_ind;
            show_ind = [];
            while cur ~= -1
                show_ind = [show_ind, cur];
%                 disp('obj state is')    
%                 [obj.vertex(cur).x, obj.vertex(cur).y, obj.vertex(cur).theta]
%                 disp('env contacts are') 
%                 obj.vertex(cur).env_contacts;
%                 disp('finger contacts are') 
%                 obj.vertex(cur).finger_contacts; 
%                 disp('-------NEXT-------')
                cur = obj.vertex(cur).parent;
            end
            show_ind = show_ind(end:-1:1);
        end

    end

end