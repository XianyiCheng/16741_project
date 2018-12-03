% evn: set of points, draw straight lines to connect in order, 2 by ? matrix
% object: set of points, the object is the convex hull of them, 2 by ? matrix
%%
env = [0,0;1000,0;1000,1000;0,1000]';
object = [25,12;-25,12;-25,-12;25,-12]';
config = [26,12,0]';
maxIter = 1000;
thr = 0.01;
cf = 0.3;
[T, isfound, path] = RRTplanning(config, pi/2, env, object, cf, maxIter, thr);
%%
figure
hold on
drawEnv(env);
hold on;
for i = 1:numel(path)
    node = T.vertex(path(i));
    config_i = [node.x, node.y, node.theta]';
    drawObject(objFrame2worldFrame(object, config_i));
    hold on;
    finger_contacts = node.finger_contacts;
    
    for j=1:size(finger_contacts,2)
    finger_contacts(1:2, j) = objFrame2worldFrame(finger_contacts(1:2, j), config_i);
    finger_contacts(1:2, j) = finger_contacts(1:2, j)/norm(finger_contacts(1:2, j));
    finger_contacts(3:4, j) = objFrame2worldFrame(finger_contacts(3:4, j), config_i);
    end
    drawContacts(finger_contacts);
    hold on;
end

axis equal
xlim([-10,150])
ylim([-10,150])
