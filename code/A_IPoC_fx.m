function d_x = f_x_A_IPoC(x, u)
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : Linearized Dynamics                                %
%                                                               %
% -------------------------- Content -------------------------- %

% [x_1 x_2 x_3 x_4 x_5 x_6] = [pos, vel, acc, theta, d_theta, dd_theta]
% [u_1 u_2] = [acc_inp jerk_inp]

[m, M, L, g, d] = sys_params();
Sx = sin(x(4)); Cx = cos(x(4));
D = (M+m*Sx^2);

[~, x_2, x_3, x_4, x_5, x_6] = deal(x(1), x(2), x(3), x(4), x(5), x(6));
[u_1, du_1] = deal(u(1), u(2));

% --------- Translations ---------- %
dx_1 = x_2;
dx_2 = (1/D)*( -m*g*Cx*Sx + m*L*x_5^2*Sx - d*x_2 + u_1 );
dx_3 = ( -m*g*x_5*cos(2*x_4) + m*L*x_5*(2*sin(x_4)*x_6 + cos(x_4)*x_5^2) - ...
    d*x_3 + du_1 )/(M + m*sin(x_4)^2) - ((m*sin(2*x_4)*x_5)*( -0.5*m*g*sin(2*x_4) + ...
    m*L*sin(x_4)*x_5^2 - d*x_2 + u_1 ))/(M + m*sin(x_4)^2)^2;

% ----------- Rotations ----------- %
dx_4 = x_5;
dx_5 = (1/(D*L))*( (m+M)*g*Sx - 0.5*m*L*x_5^2*sin(2*x_4) + Cx*(d*x_2 + u_1) );
dx_6 = ( (M + m)*g*cos(x_4)*x_5 - m*L*( sin(2*x_4)*x_5*x_6 + cos(2*x_4)*x_5^3 ) - ...
    sin(x_4)*x_5*(u_1 + d*x_2) + cos(x_4)*(d*x_3 + du_1) )/(L*(M + m*sin(x_4)^2)) - ...
( (M + m)*g*sin(x_4) - 0.5*m*L*sin(2*x_4)*x_5^2 + cos(x_4)*(u_1 + d*x_2) ) * ...
(m*x_5*sin(2*x_4))/(L*(M + m*sin(x_4)^2)^2);

d_x = [dx_1; dx_2; dx_3; dx_4; dx_5; dx_6];

% --------------------------------------------------------------- %
% ---------------- GROUND TRUTH (Do not modify!) ---------------- %
% % --------- Translations ---------- %
% dx_1 = x_2;
% dx_2 = (1/D)*( -m*g*Cx*Sx + m*L*x_5^2*Sx - d*x_2 + u_1 );
% dx_3 = ( m*g*x_5*cos(2*x_4) + m*L*x_5*(2*sin(x_4)*x_6 + cos(x_4)*x_5^2) - ...
%     d*x_3 + du_1 )/(M + m*sin(x_4)^2) - ((m*sin(2*x_4)*x_5)*( -0.5*m*g*sin(2*x_4) + ...
%     m*L*sin(x_4)*x_5^2 - d*x_2 + u_1 ))/(M + m*sin(x_4)^2)^2;
% 
% % ----------- Rotations ----------- %
% dx_4 = x_5;
% dx_5 = (1/(D*L))*( (m+M)*g*Sx - 0.5*m*L*x_5^2*sin(2*x_4) + Cx*(d*x_2 + u_1) );
% dx_6 = ( (M + m)*g*cos(x_4)*x_5 - m*L*( sin(2*x_4)*x_5*x_6 + cos(2*x_4)*x_5^3 ) - ...
%     sin(x_4)*x_5*(u_1 + d*x_2) + cos(x_4)*(d*x_3 + du_1) )/(L*(M + m*sin(x_4)^2)) - ...
% ( (M + m)*g*sin(x_4) - 0.5*m*L*sin(2*x_4)*x_5^2 + cos(x_4)*(u_1 + d*x_2) ) * ...
% (m*x_5*sin(2*x_4))/(L*(M + m*sin(x_4)^2)^2);
% 
% d_x = [dx_1; dx_2; dx_3; dx_4; dx_5, dx_6];