
function [R] = computeRotMat(n)

% write your code here
 a = [1,0];
 theta = atan(n(2)/n(1));
 R = [cos(theta),0; sin(theta),0];