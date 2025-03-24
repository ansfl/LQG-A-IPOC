function dx = fx_IPoC(x, u)
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : state vector (x_k), control input (u_k)            %
%       dx_4 (cart acceleration) is controlled directly         %
%       dx_2 (angular velocity) is controlled indirectly        %
%   Output : state vector (x_k+1)                               %
%                                                               %
% -------------------------- Content -------------------------- %

[m, M, L, g, d] = sys_params();

S_x = sin(x(3)); C_x = cos(x(3)); S_2x = sin(2*x(3));
D = (M+m*S_x^2);

dx(1,1) = x(2);
dx(2,1) = (1/D)*( -.5*m*g*S_2x + m*L*x(4)^2*S_x - d*x(4) + u);
% dx(2,1) = (1/D)*( -m*g*Cx*Sx + m*L*x(4)^2*Sx - d*x(2) + u);
dx(3,1) = x(4);
dx(4,1) = (1/(D*L))*( (m+M)*g*S_x - .5*m*L*x(4)^2*S_2x+C_x*(d*x(2)+u));
% dx(4,1) = (1/(D*L))*( (m+M)*g*Sx + Cx*(d*x(2) - m*L*x(4)^2*Sx + u) );