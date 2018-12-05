function [c,ceq] =  contactConstraint(x, dx_basis, Xnear,env, maintain_c)
theta = Xnear(3);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
dx = dx_basis*x;
dx = [R*dx(1:2); dx(3)];
Xnew = Xnear + dx;
ceq=0;
for i = 1:size(maintain_c,2)
    p = objFrame2worldFrame(maintain_c(3:4,i), Xnew);
    %[~,on] = inpolygon(p(1),p(2),env(1,:),env(2,:));
    d = p_poly_dist(p(1),p(2),env(1,:),env(2,:));
    ceq = ceq + d;
end
c = [];
%c=-norm([dx(1:2);10*dx(3)]) +0.1;
