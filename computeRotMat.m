
function [R] = computeRotMat(n)

% write your code here
 a = [1,0];
 theta = asin(n(2)/norm(n));
 if n(1) == -1 && n(2) == 0
     theta = pi;
 end
 R =[cos(theta) -sin(theta); sin(theta) cos(theta)];
