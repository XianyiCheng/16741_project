function drawEnv(env)
% Assume env to be a 2xN polygon
env = [env, env(:,1)];
plot(env(1,:), env(2,:), 'b', 'LineWidth', 2);
end