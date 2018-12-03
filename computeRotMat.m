
function [R] = computeRotMat(n)

% write your code here
 a = [1,0];
 theta = asin(n(2)/norm(n));
 R =[cos(theta) -sin(theta); sin(theta) cos(theta)];