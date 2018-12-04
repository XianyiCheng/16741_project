function [CP, CN] = randomSampleContact(object, env_p)
% randomly sample a contact point on the object
% CP: contact point, 2 by 1 vector
% CN: contact normal, pointing invard to the object convex hull
% CP and CN are in object frame
% the point shouldn't be sampled on the line segment of the object-environment contacts
n = size(object,2);

object = [object,object(:,1)];

while 1
    i = randi([1,n]);
    lineseg = [object(:,i),object(:,i+1)];
    r =rand(1);
    p = r*lineseg(:,1) + (1-r)*lineseg(:,2);
    if ~isempty(env_p) && inpolygon(p(1), p(2), [env_p(3,:),0],[env_p(4,:),0] )
        continue;
    else
        k = lineseg(:,2) - lineseg(:,1);
        n = [-k(2);k(1)];
        n = n*abs(n'*(-p))/(n'*(-p));
        break;
    end
end
CP = p;
CN = n/norm(n);
