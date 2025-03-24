function x_k_1 = f_RK45(F_x, xk, uk, dt)
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : state vector (x_k), control input (u_k)            %
%   Output : state vector (x_k+1)                               %
%                                                               %
% -------------------------- Content -------------------------- %

k1 = F_x(xk, uk);
k2 = F_x(xk + 0.5 * dt * k1, uk);
k3 = F_x(xk + 0.5 * dt * k2, uk);
k4 = F_x(xk + dt * k3, uk);
x_k_1 = xk + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);