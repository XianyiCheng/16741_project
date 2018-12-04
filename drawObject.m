function drawObject(object, color)
% Assume object to be a 2xN polygon
if nargin == 1
    color = 'r';
end
object = [object, object(:,1)];
plot(object(1,:), object(2,:), color,'LineWidth', 2)
end