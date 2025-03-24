function x_k_1 = f_Eul(xk, uk, dt)
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : state vector (x_k), control input (u_k)            %
%   Output : state vector (x_k+1)                               %
%                                                               %
% -------------------------- Content -------------------------- %

x_k_1 = xk + f_x(xk, uk)*dt;