function Xnew = extend(Xrand, Xnear)
tmax = 10;
amax = 0.0872; % max angle change: 5 degrees
dX = Xrand - Xnear;

if abs(dX(3)) > amax
    dX(3) = amax*dX(3)/abs(dX(3));
end

if norm(dX(1:2)) > tmax
    dX(1:2) = tmax*dX(1:2)/norm(dX(1:2));
end

Xnew = Xnear + dX;

end
    
