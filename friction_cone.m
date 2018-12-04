function [CPF, CNF] = friction_cone(CP, CN, friction_coeff)

N = size(CP,2);
M=2;
CPF = zeros(2, 2*N);
CNF = zeros(2,2*N);
d = [1,1;friction_coeff,-friction_coeff];
for i = 1:N
    Ri = computeRotMat(CN(:,i));
    CNF(:,((i-1)*M+1):i*M) = Ri*d;
    CPF(:, ((i-1)*M+1):i*M) = CP(:,i).*ones(2,M);
end