% test case 1
env = [1,3,3,0,0; 0,0,3,3,1];
object = [0.5,2,2,0.5; 0.5,1,3,3] - [2;2];
config = [2;2;0];
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