function matrix = GeometricDiluitionOfPrecision(params,drone,i)
%GEOMETRICDILUITIONOFPRECISION Summary of this function goes here
%   Detailed explanation goes here

G = ones(params.N_agents,4);
for idx = 1:params.N_agents
    for j = 1:3
        G(idx,j) = drone(idx).q_real(j)-drone(idx).x_est(j,i+1);
    end
end
Q = inv(G'*G);
matrix = trace(Q);

end


