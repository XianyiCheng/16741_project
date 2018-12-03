function Xrand = RandomSampleObjectConfig(env)
xmin = min(env(1,:));
xmax = max(env(1,:));
ymin = min(env(2,:));
ymax = max(env(2,:));
while true
    rand_x = rand(1)*(xmax-xmin)+xmin;
    rand_y = rand(1)*(ymax-ymin)+ymin;
    if inpolygon(rand_x, rand_y, env(1,:), env(2,:))
        break
    end
end
Xrand = [rand_x; rand_y; rand(1)*2*pi - pi];
end