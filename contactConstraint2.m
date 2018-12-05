function [c,ceq] =  contactConstraint2(x, Xnear,env, maintain_c)
theta = Xnear(3);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
ceq=0;
for i = 1:size(maintain_c,2)
    p = objFrame2worldFrame(maintain_c(3:4,i), x);
    %[~,on] = inpolygon(p(1),p(2),env(1,:),env(2,:));
    d = p_poly_dist(p(1),p(2),env(1,:),env(2,:));
    ceq = ceq + d^2;
end
c = [];
c=-norm(x-Xnear) +0.1;