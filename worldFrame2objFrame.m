function obj = worldFrame2objFrame(world_obj, config)
theta = -config(3);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
obj = R*(world_obj - config(1:2));
end