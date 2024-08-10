function k=curvature(Z,p)
    %CURVATURE Compute the curvature of a function at a point
    % INPUTS:
    % f - Function handle
    % p - point
    % OUTPUTS:
    % k - Curvature at the point p

    % Compute the gradient
    [fx,fy]= gradient(Z);

    % Compute the Hessian matrix
    [fxx,fxy]= gradient(fx);
    [fyx,fyy]= gradient(fy);    
    
    k = (fxx .* fy.^2 -fx.*fy.*(fxy + fyx) + fyy .* fx.^2) ./ ((fx.^2 + fy.^2).^(3/2));
    
    if nargin==2
        % Find the value of the gradient of a multivariate function at a
        % specified point
        x0 = p(1);
        y0 = p(2);

        % Find the index of the specified point in the grid
        [~, ix] = min(abs(x - x0));  % Closest x-index
        [~, iy] = min(abs(y - y0));  % Closest y-index
        
        % Get the curvature at the specified point
        curvature_at_point = k(iy, ix);  % Get the curvature value
        k = curvature_at_point;
    end
    
end