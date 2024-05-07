function k=curvature(z)
    %CURVATURE Compute the curvature of a function at a point
    % INPUTS:
    % f - Function handle
    % OUTPUTS:
    % k - Curvature at the point p

    % Compute the gradient
    [fx,fy]= gradient(z);

    % Compute the Hessian matrix
    [fxx,fxy]= gradient(fx);
    [fyx,fyy]= gradient(fy);    
    
    %k= abs(fxx .* fy.^2 - 2 * fx .* fy .* fxy + fyy .* fx.^2) ./ ((fx.^2 + fy.^2).^(3/2));
    
    k = abs(fx.*fyy-fxx.*fy)/((fx.^2)+(fy.^2)).^(3/2);
    
end