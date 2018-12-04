function Xnew = extend(Xrand, Xnear, env_contacts)

angle_sign = sign(Xrand(3));
tmax = 5;
amax = 0.0872; % max angle change: 5 degrees
if Xnear(3) > pi
    Xnear(3) = -2*pi + Xnear(3);
end
if Xnear(3) < -pi
    Xnear(3) = Xnear(3) + 2*pi;
end
dX = Xrand - Xnear;

if abs(dX(3)) > amax
    dX(3) = amax*dX(3)/abs(dX(3));
end

if norm(dX(1:2)) > tmax
    dX(1:2) = tmax*dX(1:2)/norm(dX(1:2));
end
dx = dX;

if isempty(env_contacts)
    dx =dX;
else
    % convert contacts back to obj frame
theta = Xnear(3);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

env_contacts(1:2, :) = R'*env_contacts(1:2, :);
env_contacts(3:4, :) = R'*(env_contacts(3:4,:)- Xnear(1:2));
N = size(env_contacts,2);    
cwset = nchoosek([1:N],randi([1,N]));
cwset = cwset(randi([1,size(cwset,1)]),:);
%cwset = [3,4];
maintain_c = env_contacts(:,cwset);
leave_c = env_contacts;
leave_c(:,cwset)=[];
[cw] = contactScrew2D(maintain_c(3:4,:),maintain_c(1:2,:));
dx_basis = null(cw');
if numel(dx_basis) ==0
    dx = dX;
else
    if ~isempty(leave_c)
        cwgeq = contactScrew2D(leave_c(3:4,:),leave_c(1:2,:));
        for j = 1:1000
            c = randn(1,size(dx_basis,2));
            dx = sum(dx_basis.*[c;c;c],2);
            if sum(cwgeq'*dx <= 0) == 0 && sign(dx(3)) == angle_sign 
                break
            end
            if j == 1000
                dx = dx_basis(:,1);
                dx = dx*sign(dx(3))*angle_sign;
            end
        end

    else
        c = randn(1,size(dx_basis,2));
        dx = sum(dx_basis.*[c;c;c],2);
    end
    g = [R,Xnear(1:2); 0,0,1];

    %dx = g*dx*pinv(g);
    %dx = [R*dx(1:2); cross2D(dx(1:2), Xnear(1:2)) + dx(3)];
    dx = [R*dx(1:2); dx(3)];
    %dx = [dx(2:3);dx(1)];
end
end
% if ~isempty(env_contacts)
%     %fun = @(x)(dX(1)-x(1))^2 + (dX(2) - x(2))^2 + (dX(3)-x(3))^2;
%     fun = @(x)(dX(3)-x(3))^2;
%     [cw] = contactScrew2D(env_contacts(3:4,:),env_contacts(1:2,:));
%     ind_eq = randi([1,size(cw,2)]);
%     A = cw;
%     A(:,ind_eq)=[];
%     b = zeros(size(cw,2)-1,1);
%     Aeq = cw(:,ind_eq);
%     beq=0;
%     dXc = fmincon(fun,dX,A',b,Aeq',beq);
% 
%     Xnew = Xnear + dXc;
% else
%    Xnew = Xnear + dX;
Xnew = Xnear + dx;
[cw] = contactScrew2D(env_contacts(3:4,:),env_contacts(1:2,:));
dx_basis = null(cw');
if numel(dx_basis) ==0
    Xnew = Xnear + 0;
else
    dx = dx_basis;
    dx = [R*dx(1:2); dx(3)];
    Xnew = Xnear + dx*sign(dx(3))*angle_sign;
end
end
    
