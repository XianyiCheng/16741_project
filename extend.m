function Xnew = extend(Xrand, Xnear, env_contacts)
tmax = 10;
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

if ~isempty(env_contacts)
    fun = @(x)(dX(1)-x(1))^2 + (dX(2) - x(2))^2 + (dX(3)-x(3))^2;
    [cw] = contactScrew2D(env_contacts(3:4,:),env_contacts(1:2,:));
    ind_eq = randi([1,size(cw,2)]);
    A = cw;
    A(:,ind_eq)=[];
    b = zeros(size(cw,2)-1,1);
    Aeq = cw(:,ind_eq);
    beq=0;
    dXc = fmincon(fun,dX,A',b,Aeq',beq);

    Xnew = Xnear + dXc;
else
    Xnew = Xnear + dX;

end
    
