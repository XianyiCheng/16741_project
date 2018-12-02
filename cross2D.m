function cp2 = cross2D(CP, CN)
% input: 2 x n
% output: 1xn
if sum(size(CP)==1)>0
    cp2 = CP(1).*CN(2) - CP(2).*CN(1);
else
    cp2 = CP(1,:).*CN(2,:) - CP(2,:).*CN(1,:);
end
end