% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : N/A                                                %
%   Output : Initial conditions                                 %
%                                                               %
% -------------------------- Content -------------------------- %

if pos == 1                                  % Upward
    x_0   = [-3; 0.2; pi-0.2; -0.1];         % Initial conditions
    x_ref = [0; 0; pi; 0];                   % Desired setpoint(s)
elseif pos == -1                             % Bottom
    x_0   = [-3; 0.2; -0.2; -0.1];               
    x_ref = [0; 0; 0; 0];
end

% -------------- Control law --------------- %
u_bnd = 30;                                  % Max control input +/- 3g [m/s^2]
w_r = @(x)( exp(-x/5).*sin(x*2) )*0.01;      % Error model (within local linearity !)
u_sat = @(u_x) max(-u_bnd, min(u_bnd, u_x)); % Control law
u_inp = @(x) u_sat( -K*( x - x_ref + w_r(x) ) ); % Apply saturation to control input
% u_inp = @(x) -K*( x - x_ref ); % Simple rule

% Initialization
u_k = zeros(1, length(tt)); x_std = zeros(n_dim, length(tt)); y_k = x_std;
x_k = x_std; x_k(:,1) = x_0;
x_e = x_std; x_e(:,1) = x_0;

v_idx = [1,2,3,4]';                          % Full obsverability
w_k = @() randn( size(A*x_k(:,1)) )*std_w;
v_k = @() randn( size(v_idx) )*std_v;

% Solution loop
A_k = expm(A*dt); B_k = B*dt;                % Discrete-time solution
P_k = 0.1*eye(n_dim);                        % Initialize error covariance