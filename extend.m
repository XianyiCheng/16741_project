function Xnew = extend(Xrand, Xnear)
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
dX = dX*(-rand(1)+0.5)*2;

Xnew = Xnear + dX;

end
    
