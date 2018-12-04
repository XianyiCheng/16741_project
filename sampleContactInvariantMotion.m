function dx = sampleContactInvariantMotion(env_contacts)

    %fun = @(x)(dX(1)-x(1))^2 + (dX(2) - x(2))^2 + (dX(3)-x(3))^2;
    [cw] = contactScrew2D(env_contacts(3:4,:),env_contacts(1:2,:));
    dx_basis = null(cw');
    dx = sum(dx_basis.*randn(size(dx_basis,2)),2);

end
    
