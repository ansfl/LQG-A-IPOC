function [A_s, B_s] = Linearize_A_IPoC( pos )
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : NA                                                 %
%   Output : Linearized Dynamics                                %
%                                                               %
% -------------------------- Content -------------------------- %

% [x_1 x_2 x_3 x_4 x_5] = [pos, vel, acc, theta, d_theta, dd_theta]

syms x_1 x_2 x_3 x_4 x_5 x_6 u_1 du_1 m M L g d

Sx = sin(x_4); Cx = cos(x_4);
D = (M+m*Sx^2);

% --------- Translations ---------- %
dx_1 = x_2;
dx_2 = (1/D)*( -m*g*Cx*Sx + m*L*x_5^2*Sx - d*x_2 + u_1 );
dx_3 = ( m*g*x_5*cos(2*x_4) + m*L*x_5*(2*sin(x_4)*x_6 + cos(x_4)*x_5^2) - ...
    d*x_3 + du_1 )/(M + m*sin(x_4)^2) - ((m*sin(2*x_4)*x_5)*( -0.5*m*g*sin(2*x_4) + ...
    m*L*sin(x_4)*x_5^2 - d*x_2 + u_1 ))/(M + m*sin(x_4)^2)^2;

% ----------- Rotations ----------- %
dx_4 = x_5;
dx_5 = (1/(D*L))*( (m+M)*g*Sx - 0.5*m*L*x_5^2*sin(2*x_4) + Cx*(d*x_2 + u_1) );
dx_6 = ( (M + m)*g*cos(x_4)*x_5 - m*L*( sin(2*x_4)*x_5*x_6 + cos(2*x_4)*x_5^3 ) - ...
    sin(x_4)*x_5*(u_1 + d*x_2) + cos(x_4)*(d*x_3 + du_1) )/(L*(M + m*sin(x_4)^2)) - ...
( (M + m)*g*sin(x_4) - 0.5*m*L*sin(2*x_4)*x_5^2 + cos(x_4)*(u_1 + d*x_2) ) * ...
(m*x_5*sin(2*x_4))/(L*(M + m*sin(x_4)^2)^2);

f_x = [dx_1; dx_2; dx_3; dx_4; dx_5; dx_6];
x_t = [x_1, x_2, x_3, x_4, x_5, x_6];
u_t = [u_1, du_1];

if pos == 1
    x_eq = [0, 0, 0, pi, 0, 0];            % Upward equilibrium point
elseif pos == -1
    x_eq = [0, 0, 0, 0, 0, 0];             % Bottom equilibrium point
end
u_eq = [0 0];                              % Control inputs @ equilibrium point

% Compute Jacobians (symbolic)
J_A = jacobian( f_x, x_t);
J_B = jacobian( f_x, u_t);

% Compute Jacobians (equilibrium points)

A_sym = subs(J_A, u_t, u_eq);
A_sym = subs(A_sym, x_t, x_eq);

B_sym = subs(J_B, u_t, u_eq);
B_sym = subs(B_sym, x_t, x_eq);

% Compute Jacobians (numerical values)
[m_s, M_s, L_s, g_s, d_s] = sys_params();
A_s = double(subs(A_sym, [m M L g d], [m_s, M_s, L_s, g_s, d_s]));
B_s = double(subs(B_sym, [m M L], [m_s M_s, L_s]));