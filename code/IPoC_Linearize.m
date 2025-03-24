function [A_s, B_s] = Linearize_IPoC( pos )
% ------------------------ Description ------------------------ %
%                                                               %
%   Desc.  : Validate Brunton's nonlinear function              %
%   Input  : NA                                                 %
%   Output : Linearized Dynamics                                %
%                                                               %
% -------------------------- Content -------------------------- %

syms x_1 x_2 x_3 x_4 u_1 m M L g d

Sx = sin(x_3); Cx = cos(x_3);
D = (M+m*(1-Cx^2));

% Dynamical system model ( dx = f(x,u) )
dx_1 = x_2;
dx_2 = (1/D)*( -m*g*Cx*Sx + m*L*x_4^2*Sx - d*x_2 + u_1 );
dx_3 = x_4;
dx_4 = (1/(D*L))*( (m+M)*g*Sx + Cx*(d*x_2 - m*L*x_4^2*Sx + u_1 ) );

f_x = [dx_1; dx_2; dx_3; dx_4];
x_t = [x_1, x_2, x_3, x_4];

if pos == 1
    x_eq = [0, 0, pi, 0];            % Upward equilibrium point
elseif pos == -1
    x_eq = [0, 0, 0, 0];             % Bottom equilibrium point
end

% Compute Jacobians (symbolic)
J_A = jacobian( f_x, [x_1, x_2, x_3, x_4]);
J_B = jacobian( f_x, u_1);

% Compute Jacobians (equilibrium points)
A_sym = subs(J_A, x_t, x_eq);
B_sym = subs(J_B, x_t, x_eq);

% Compute Jacobians (numerical values)
[m_s, M_s, L_s, g_s, d_s] = sys_params();
A_s = double(subs(A_sym, [m M L g d], [m_s, M_s, L_s, g_s, d_s]));
B_s = double(subs(B_sym, [M L], [M_s, L_s]));