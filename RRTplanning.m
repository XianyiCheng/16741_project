function [T_   , isfound, goal_path] = RRTplanning(Xstart, Xgoal, env, object, friction_coeff, maxIter, thr)
    % T: tree of RTT planning
    % Xstart: start configuration of object, [x,y,theta]
    % Xgoal: goal configuration of object, only [theta]
    % maxIter: maximum iteration number
    % thr: the threshold allow for Xgoal
    %
    isfound = 0;
    end_ind = 0;
    goal_path = [];
    [isStartCollide,start_contacts] = CollisionDetection(env, object, Xstart);
    if isStartCollide 
        error('start configuration collided!');
    end
    if isempty(start_contacts)
        error('The object is floating :-) ');
    end
    %Start to construct RTT tree
    T = RRTtree(Xstart, start_contacts, []);

    for i = 1:maxIter
        
        if mod(i,3) == 1
            Xrand = Xgoal;
        elseif mod(i,3) == 2
            Xrand = Xgoal-2*pi;
        else
            Xrand = RandomSampleObjectConfig(env); % TODO: sample from random state,50% from the goal stat
        end
        
        [Xnear_ind, ~] = T.nearestNeighbor(Xrand); 
        Xnear = [T.vertex(Xnear_ind).x, T.vertex(Xnear_ind).y, T.vertex(Xnear_ind).theta]';
        if numel(Xrand) == 1
            Xnew = extend([Xnear(1:2);Xrand], Xnear);
        else
            Xnew = extend(Xrand, Xnear); 
        end
        [isXnewCollide,Xnew_env_contacts] = CollisionDetection(env, object, Xnew);
        if isXnewCollide
            continue;
        end
        
        [isXnewMotion, Xnew_finger_contacts] = isStableMotionAllowed(Xnew-Xnear, ...
            object,T.vertex(Xnear_ind).env_contacts,T.vertex(Xnear_ind).finger_contacts, Xnear, [Xnear(1:2);0;1], friction_coeff); 
        % TODO
        
        if ~isXnewMotion
            continue;
        end
        
        % combining adding vertex and edge
        T = T.add_node(Xnear_ind, Xnew, Xnew_env_contacts, Xnew_finger_contacts);
        [Xclosest_ind, cur_dist] = T.nearestNeighbor(Xgoal);
        if cur_dist < thr % TODO: check if goal is reached
            isfound = 1;
            end_ind = Xclosest_ind;
            break;
        end
    end
    if end_ind ~= 0
        % display states and fingers
        goal_path = T.get_path(end_ind);
        T_ = T;
    else
        T_ = T;
        fprintf('no state close enough to goal state\n');
    end
end