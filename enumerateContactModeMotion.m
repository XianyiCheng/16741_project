function Xnew_set = enumerateContactModeMotion(Xrand, Xnear, env, env_contacts)

count = 1;
% constrain random config to be small
tmax = 2;
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

% enumerate all contact modes
if ~isempty(env_contacts)
theta = Xnear(3);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
env_contacts(1:2, :) = R'*env_contacts(1:2, :);
env_contacts(3:4, :) = R'*(env_contacts(3:4,:)- Xnear(1:2));

N = size(env_contacts,2);

for i = 1:N
    cwset = nchoosek([1:N],i);
    for k = 1:size(cwset,1)
        cwset_i = cwset(k,:);
        maintain_c = env_contacts(:,cwset_i);
        leave_c = env_contacts;
        leave_c(:,cwset_i)=[];
        [cw] = contactScrew2D(maintain_c(3:4,:),maintain_c(1:2,:));
        dx_basis = null(cw');
        if isempty(dx_basis)
            continue;
        end
        fun = @(x)norm(dx_basis*x - dX);
        x0 = ones(size(dx_basis,2),1)/10;
%        if ~isempty(leave_c)
%            cwgeq = contactScrew2D(leave_c(3:4,:),leave_c(1:2,:));
 %           x = fmincon(fun,x0,-cwgeq'*dx_basis,zeros(size(cwgeq,2),1),[],[],[],[],...
 %               @(x) contactConstraint(x,dx_basis, Xnear,env, maintain_c));
%        else
            x = fmincon(fun,x0,[],[],[],[],[],[],...
                @(x) contactConstraint(x,dx_basis, Xnear,env, maintain_c));
%        end
        dx = dx_basis*x;
        dx = [R*dx(1:2); dx(3)];
        if norm(dx) ~=0
            Xnew_set{count} = Xnear + dx;
            count = count+1;
        end
    end
end
else
    Xnew_set = {};
end
            
            