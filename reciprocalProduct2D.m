function rp = reciprocalProduct2D(wrench, twist)
% wrench 3x n, n wrenchs/contacts, (fx, fy, n)
% twist 3 x 1, 1 twist, (w, vx, vy)

rp = wrench(1:2,:)'*twist(2:3)+ wrench(3,:)'*twist(1);

end