function [T, isfound] = RRTplanning(Xstart, Xgoal, env, object, maxIter, thr)
% T: tree of RTT planning
% Xstart: start configuration of object, [x,y,theta]
% Xgoal: goal configuration of object, only [theta]
% maxIter: maximum iteration number
% thr: the threshold allow for Xgoal

%
isfound = 0;
[isStartOk,start_contacts] = CollisionDetection(env, object, Xstart);
if ~isStartOk 
    error('start configuration collided!');
end
if isempty(start_contacts)
    error('The object is floating :-) ');
end

%Start to construct RTT tree
T = RTTtree(X_start, start_contacts, []);

for i = 1:maxIter
    if 1% TODO: check if goal is reached
        isfound = 1;
        break;
    end 
    
    if mod(i,2) == 0
        Xrand = Xgoal;
    else
        Xrand = RandomSampleObjectConfig(env); % TODO: sample from random state,50% from the goal stat
    end
    
    Xnear = T.nearestNeighbor(Xrand); %TODO: in RRT tree class
    Xnew = extend(Xrand, Xnear); % TODO
    [isXnewOk,Xnew_env_contacts] = CollisionDetection(env, object, Xnew);
    if ~isXnewOk
        continue;
    end
    
    [isXnewMotion, Xnew_finger_contacts] = isStableMotionAllowed( ) % TODO
    
    if ~isXnewMotion
        continue;
    end
    
    T.addVertex(Xnew, Xnew_env_contacts, Xnew_finger_contacts); %TODO
    T.addEdge(Xnear, Xnew); %TODO
    
    
end

    


