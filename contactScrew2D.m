function [CW] = contactScrew2D(CP, CN)
% CP: a set of contact point positions [[pix; piy] ...]; 2xN matrix
% CN: a set of inward-pointing directions of contact normals  2xN matrix
% CW: a set of normalized contact screws
N = size(CP,2);

CN = CN./vecnorm(CN,2,1);
CW = zeros(3,N);
CW(1:2,:) = CN;
CW(3,:) = CP(1,:).*CN(2,:) - CP(2,:).*CN(1,:);
end

