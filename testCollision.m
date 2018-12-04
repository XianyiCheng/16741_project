%%
% test case 1
env = [1,3,3,0,0; 0,0,3,3,1];
object = [0.5,2,2,0.5; 0.5,1,3,3] - [2;2];
config = [2;2;0];%RandomSampleObjectConfig(env);%[2;2;0];
figure
hold on
drawEnv(env);
drawObject(objFrame2worldFrame(object, config));
[bCD, contacts] = CollisionDetectionV2(env, object, config);
drawContacts(contacts)

% convert contacts back to obj frame
for i=1:size(contacts,2)
    contacts(1:2, i) = worldFrame2objFrame(contacts(1:2, i), config);
    contacts(3:4, i) = worldFrame2objFrame(contacts(3:4, i), config);
end
%%
% test case 2
env = [0,3,3,0; 0,0,3,3];
object = [0,2,2,0; 0,0,2,2];
config = [0;0;0];
figure
hold on
drawEnv(env);
drawObject(objFrame2worldFrame(object, config));
[bCD, contacts] = CollisionDetectionV2(env, object, config);
drawContacts(contacts)

% convert contacts back to obj frame
for i=1:size(contacts,2)
    contacts(1:2, i) = worldFrame2objFrame(contacts(1:2, i), config);
    contacts(3:4, i) = worldFrame2objFrame(contacts(3:4, i), config);
end
%%
% test case 3
env = [0,0;1000,0;1000,1000;0,1000]';
%object = [25,12;-25,12;-25,-12;25,-12]';
object = [12,12;-12,12;-12,-12;12,-12]';

a = 30*pi/180;
config = [24*sin(a) + sqrt(2*12^2)*cos(a+pi/4),sqrt(2*12^2)*sin(a+pi/4), a]';
figure
hold on
drawEnv(env);
drawObject(objFrame2worldFrame(object, config));
[bCD, contacts] = CollisionDetectionV2(env, object, config);
drawContacts(contacts)

% convert contacts back to obj frame
for i=1:size(contacts,2)
    contacts(1:2, i) = worldFrame2objFrame(contacts(1:2, i), config);
    contacts(3:4, i) = worldFrame2objFrame(contacts(3:4, i), config);
end
%
% test case 3
env = [0,0;1000,0;1000,1000;0,1000]';
%object = [25,12;-25,12;-25,-12;25,-12]';
object = [12,12;-12,12;-12,-12;12,-12]';
config = [12,12,3e-9]';
%a = 30*pi/180;
%config = [24*sin(a) + sqrt(2*12^2)*cos(a+pi/4),sqrt(2*12^2)*sin(a+pi/4), a]';
figure
hold on
drawEnv(env);
drawObject(objFrame2worldFrame(object, config));
[bCD, contacts] = CollisionDetectionV2(env, object, config);
drawContacts(contacts)