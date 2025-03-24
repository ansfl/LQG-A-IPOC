function C_k = h_x(v, n_dim)
% ------------------------ Description ------------------------ %
%   Observation function, determines the system output          %
%   Input  : state vector (x_k), control input (u_k)            %
%   Output : state vector (x_k+1)                               %
%                                                               %
% -------------------------- Content -------------------------- %

C_n = eye(n_dim);
C_k = C_n(v',:);