function drawObject(object)
% Assume object to be a 2xN polygon
object = [object, object(:,1)];
plot(object(1,:), object(2,:), 'r')
end