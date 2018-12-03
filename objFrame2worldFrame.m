function world_obj = objFrame2worldFrame(obj, config)
theta = config(3);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
world_obj = R*obj + config(1:2);
end