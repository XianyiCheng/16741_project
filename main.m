% evn: set of points, draw straight lines to connect in order, 2 by ? matrix
% object: set of points, the object is the convex hull of them, 2 by ? matrix
%%
env = [0,0;1000,0;1000,1000;0,1000]';
%object = [25,12;-25,12;-25,-12;25,-12]';
object = [12,12;-12,12;-12,-12;12,-12]';
%config = [12,12,0]';
%a = 30*pi/180;
%goal= [24*sin(a) + sqrt(2*12^2)*cos(a+pi/4),sqrt(2*12^2)*sin(a+pi/4), a]';
%config = [15,12,0]';
%goal = [12,12,0]';
config = [12*sqrt(2),12*sqrt(2),pi/4]';
goal = [12,12,0]';
%goal = 0.01*[25,12,pi/4*100]';
maxIter = 1000;
thr = 0.1;
cf = 1;
[T, isfound, path] = RRTplanning(config, goal, env, object, cf, maxIter, thr);
%%
figure
hold on
drawEnv(env);
hold on;
for i = 1:numel(path)
    node = T.vertex(path(i));
    config_i = [node.x, node.y, node.theta]';
    if i == 1
    drawObject(objFrame2worldFrame(object, config_i), 'm');
    elseif i == numel(path)
        drawObject(objFrame2worldFrame(object, config_i), 'g');
    else
        drawObject(objFrame2worldFrame(object, config_i));
    end
    hold on;
    finger_contacts = node.finger_contacts;
    
    for j=1:size(finger_contacts,2)
    finger_contacts(1:2, j) = objFrame2worldFrame(finger_contacts(1:2, j), config_i);
    finger_contacts(1:2, j) = finger_contacts(1:2, j)/norm(finger_contacts(1:2, j));
    finger_contacts(3:4, j) = objFrame2worldFrame(finger_contacts(3:4, j), config_i);
    end
    drawContacts(finger_contacts);
    %drawContacts(T.vertex(i).env_contacts);
    hold on;
end

axis equal
xlim([-10,150])
ylim([-10,150])
