function [X_e, P_e] = KF_update(x_e, P_k, y_res, C_k, R)
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : state vector (x_k), control input (u_k)            %
%   Output : state vector (x_k+1)                               %
%                                                               %
% -------------------------- Content -------------------------- %

S_k = C_k*P_k*C_k' + C_k*R*C_k';            % Innovation covariance (adjustable)
K_k = P_k*C_k' * (inv(S_k));                % Kalman gain

X_e = x_e + K_k*y_res;                      % State vector update
P_e = ( eye(size(x_e,1)) - K_k*C_k ) * P_k; % Stace covariance update