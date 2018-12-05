function Xnew_set = enumerateContactModeMotion2(Xrand, Xnear, env, env_contacts)

count = 1;
% constrain random config to be small
tmax = 3;
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

for i = N:N
    cwset = nchoosek([1:N],i);
    for k = 1:size(cwset,1)
        cwset_i = cwset(k,:);
        maintain_c = env_contacts(:,cwset_i);
        fun = @(x)norm(x - Xrand);
        x0 = Xnear;
        x = fmincon(fun,x0,[],[],[],[],[],[],...
            @(x) contactConstraint2(x,Xnear,env, maintain_c));
%        end
        Xnew_set{count} = x;
        count = count + 1;
    end
end
else
    Xnew_set = {};
end