function [bMA, final_finger_contacts] = isStableMotionAllowed(twist, object, env_contacts, finger_contacts, X, external_force, friction_coeff)
% object: points
% twist: the to-be test motion 3x1, 
% env_contacts: 4 x ? matrix, in the object frame [point, normal; ...]'
% finger_contact: 4 x ? matrix, in the object frame
% X = object configuration to the world frame
% external force,4xn in the world frame, act point, force direction*amplitute
bMA = false;
final_finger_contacts=[];

if ~isempty(env_contacts)
env_p = env_contacts(3:4,:);
env_n = env_contacts(1:2,:);

env_cw = contactScrew2D(env_p, env_n);
env_rp = reciprocalProduct2D(env_cw, twist);
 
sticking_contacts = [];

% for every env contact, test their constraint to this motion
if sum(env_rp<0) > 0
    %bMA = false;
    %final_finger_contacts=[];
    return
end
    
env_active_p = env_p(:, env_rp==0);
env_active_n = env_n(:, env_rp==0);

%for every active env contact, compute the object velocity on this point
% vp = v0 + w x p = v0 + [-wy, wx]
env_active_vp = twist(2:3) + twist(1).*[-env_active_p(2,:); env_active_p(1,:)];
flag = -cross2D(env_active_n, env_active_vp);

% if flag == 0, add to sticking contact
sticking_contacts = [sticking_contacts,[env_active_p(:, flag == 0);env_active_n(:, flag == 0)]];
sticking_wrenches = contactScrew2D(friction_cone(sticking_contacts(3:4,:), sticking_contacts(1:2,:),friction_coeff));
% if flag > 0, contact providing sliding wrench on the left of its normal
sliding_wrenches = [];
left_slide = find(flag>0);
for i = left_slide
    force = computeRotMat(env_active_n(:, i))*[1; friction_coeff];
    wrench = [force;cross2D(force, env_active_p(:,i))];
    sliding_wrenches = [sliding_wrenches, wrench];
end

% if flag < 0, contact providing sliding wrench on the right of its normal
right_slide = find(flag<0);
for i = right_slide
    force = computeRotMat(env_active_n(:, i))*[1; -friction_coeff];
    wrench = [force;cross2D(force, env_active_p(:,i))];
    sliding_wrenches = [sliding_wrenches, wrench];
end

else
    env_p =[];
    env_n=[];
    sliding_wrenches=[];
    sticking_wrenches=[];
end

% compute external wrenches in object frame
R = [cos(X(3)+twist(3)), sin(X(3)+twist(3)); sin(X(3)+twist(3)), -cos(X(3)+twist(3))];
ext_p_pos = R'*(external_force(1:2,:) - X(1:2));
ext_f_pos = R'*external_force(3:4,:);
ext_f_pos = ext_f_pos/norm(ext_f_pos);
external_wrenches_pos = [ext_f_pos; cross2D(ext_f_pos, ext_p_pos)];

R = [cos(X(3)), sin(X(3)); sin(X(3)), -cos(X(3))];
ext_p = R'*(external_force(1:2,:) - X(1:2));
ext_f = R'*external_force(3:4,:);
external_wrenches = [ext_f; cross2D(ext_f, ext_p)];

if ~isempty(finger_contacts)
    finger_p = finger_contacts(3:4,:);
    finger_n = finger_contacts(1:2,:);
    [fps,fns]=friction_cone(finger_p, finger_n,friction_coeff);
    finger_wrenches = contactScrew2D(fps,fns);
    CW = [finger_wrenches, sliding_wrenches, sticking_wrenches];
    [bWC_pre, ~] = isWrenchCompensate(CW, sum(external_wrenches,2));
    [bWC_pos, ~] = isWrenchCompensate(CW, sum(external_wrenches_pos,2));
    bWC = bWC_pre & bWC_pos;
else
    % the start configuration
    for finger_sample = 1:100
        [fp1, fn1] = randomSampleContact(object, env_contacts);
        [fp2, fn2] = randomSampleContact(object, env_contacts);
        [fps, fns] = friction_cone([fp1,fp2], [fn1,fn2],friction_coeff);
        finger_wrenches = contactScrew2D(fps,fns);
        CW = [finger_wrenches, sliding_wrenches, sticking_wrenches];
        [bWC_pre, ~] = isWrenchCompensate(CW, sum(external_wrenches,2));
        [bWC_pos, ~] = isWrenchCompensate(CW, sum(external_wrenches_pos,2));
        bWC = bWC_pre & bWC_pos;
        if bWC
            fprintf('find fingers')
            finger_contacts = [fn1,fn2;fp1,fp2];
            break
        end
    end
    if ~bWC
        return
    end
end

if bWC
    bMA = true;
    final_finger_contacts = finger_contacts;
    return
end

% if not compensated, sampling for possible switching contacts
n_finger = size(finger_contacts,2);
[eps,ens] = friction_cone(env_p, env_n,friction_coeff);
env_contact_wrenches = contactScrew2D(eps,ens);
for i = 1:n_finger
    rest_finger_wrenches = finger_wrenches;
    rest_finger_wrenches(:, ((i-1)*2+1):i*2) = [];
    [rbWC, ~] = isWrenchCompensate([rest_finger_wrenches,env_contact_wrenches], sum(external_wrenches,2));
    if rbWC % sample for possible finger contacts 
        for iter = 1:100
            [sp, sn] = randomSampleContact(object, env); %TODO
            s_wrenches = contactScrew2D(friction_cone(sp, sn,friction_coeff));
            s_bWC = isWrenchCompensate([finger_wrenches,s_wrenches, sliding_wrenches, sticking_wrenches], sum(external_wrenches,2));
            if s_bWC
                bMA = true;
                finger_contacts(i,:) = [sp;sn];
                final_finger_contacts = finger_contacts;
                return
            end
        end
    end
end

end
    

    







