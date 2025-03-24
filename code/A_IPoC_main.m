clc; clear; close all;

set(0, 'DefaultFigureRenderer', 'painters')
set(0, 'defaultfigurecolor', [1 1 1]);
set(0, 'DefaultAxesXGrid', 'on');
set(groot, 'DefaultLineLineWidth', 2);
set(0, 'DefaultTextInterpreter', 'latex');  % For all text objects (titles, labels, etc.)
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');  % For axis tick labels
set(0, 'DefaultLegendInterpreter', 'latex');  % For legends

fig_loc = [1500 380 550 500];
Fig = @() figure('rend', 'painters', 'pos', fig_loc);

%% Execution CFG
pos = 1;                    % Upright position (pos=-1 for bottom)
dt = 0.005; t_f = 15; tt = 0:dt:t_f;
t_go = 1 - tt/tt(end);              % Time normalization
rho = 1;                    % Update : prediction ratio

[m_s, M_s, L_s, ~, ~] = sys_params();
[std_w, std_v] = deal(0.0005, 0.0005);
Q_func = @(n_dim) diag(ones(n_dim,1));

[A, B] = A_IPoC_Linearize( pos ); n_dim = size(A, 1);
Q_lqr = Q_func(n_dim); R_lqr = diag(1/10);
K = lqr(A, B, Q_lqr, R_lqr);
Q = diag( rand(n_dim,1) )*std_w;             % Process noise covariance
R = diag( rand(n_dim,1) )*std_v;             % Measure noise covariance

A_IPoC_Init;                            % Initial conditions
A_IPoC_Solve;                           % Solution loop

%% Animation
close; 
k_mean = 10; fps = 10;

for k=1:fps:length(tt)
    f_visualization(x_k(:,k));
    if (k>k_mean) % non-dynamics termination || kinematic divergence
        if (abs( mean(x_k(2,k-k_mean:k)) ) < 0.0001) || (abs( mean(x_k(1, k-k_mean:k)) ) > 15)
            pause(0.5); break;
        end
    end
    clc; fprintf('%.2f', k*dt);
end

%% States vs. time
figure; 
subplot(3, 2, 1); hold on; 
plot(tt, x_k(1,:)); 
plot(tt, x_e(1,:), color='g', linestyle='--');
ylabel('$x(t)$', fontsize=13);
% title('True states vs Estimates', fontsize=14);

subplot(3, 2, 2); hold on; 
plot(tt, x_k(4,:)); 
plot(tt, x_e(4,:), color='g', linestyle='--');
ylabel('$\theta(t)$', fontsize=13);

subplot(3, 2, 3); hold on; 
plot(tt, x_k(2,:)); 
plot(tt, x_e(2,:), color='g', linestyle='--');
ylabel('$\dot{x}(t)$', fontsize=13)

subplot(3, 2, 4); hold on; 
plot(tt, x_k(5,:)); 
plot(tt, x_e(5,:), color='g', linestyle='--');
ylabel('$\dot{\theta}$(t)', fontsize=13);

subplot(3, 2, 5); hold on; 
plot(tt, x_k(3,:)); 
plot(tt, x_e(3,:), color='g', linestyle='--');
ylabel('$\ddot{x}(t)$', fontsize=13);
xlabel('Time [s]', fontsize=12); 

subplot(3, 2, 6); hold on; 
plot(tt, x_k(6,:)); 
plot(tt, x_e(6,:), color='g', linestyle='--');
ylabel('$\ddot{\theta}$(t)', fontsize=13);
xlabel('Time [s]', fontsize=12); 

% axesHandles = findall(gcf, 'Type', 'axes');
% linkaxes(axesHandles, 'x');
% xlim([3.3, 3.42]);

lgd = legend('$x(t)$', '$\hat{x}(t)$', 'fontsize', 13, 'Orientation', 'horizontal');
set(lgd, 'Position', [0.47, 0.0, 0.1, 0.08]);
% set(lgd, 'Position', [0.5, 0.875, 0.1, 0.1]);

% exportgraphics(gcf, 'Fig_states_1.png', 'Resolution', 300); % 300 DPI for high resolution

%% Temporal Analysis
clc;
t_len = size(x_k,2);
t_0 = 50; tt_0 = tt(t_0:end);

disp('Position analysis:');
step_pos = stepinfo(x_k(1,t_0:end), tt_0);
disp([step_pos.PeakTime, step_pos.TransientTime, step_pos.SettlingTime]);

disp('Angle analysis:');
step_ang = stepinfo(x_k(4,:), tt);
disp([step_ang.PeakTime, step_ang.TransientTime, step_ang.SettlingTime]);


%% States vs. time
% figure; 
% subplot(3, 3, 1); hold on; dx_1 = x_k(1,:) - x_e(1,:);
% plot(tt, dx_1); ylabel('$\delta x(t)$', fontsize=13);
% 
% subplot(3, 3, 2); hold on; dx_2 = x_k(4,:) - x_e(4,:);
% plot(tt, dx_2); ylabel('$\delta \theta(t)$', fontsize=13);
% 
% subplot(3, 3, 3); hold on; 
% plot(tt, u_k(1,:)); ylabel('$u(t)$', fontsize=13);
% 
% 
% axesHandles = findall(gcf, 'Type', 'axes');
% linkaxes(axesHandles, 'x');
% xlim([0, 1.5]);

% lgd = legend('$x(t)$', '$\hat{x}(t)$', 'fontsize', 13, 'Orientation', 'horizontal');
% set(lgd, 'Position', [0.72, 0.01, 0.1, 0.08]);

%% 
% figure; 
% dx_1 = x_k(1,:)-x_e(1,:);
% subplot(4, 1, 1); hold on; plot(tt, dx_1);
% plot(tt, dx_1+x_std(1,:), tt, dx_1-x_std(1,:), Color='r', linestyle='--'); 
% ylabel('$\delta \hat{x}(t)$ [m]', fontsize=13);
% title('Error states \& Uncertainties', fontsize=14);
% 
% dx_2 = x_k(2,:)-x_e(2,:);
% subplot(4, 1, 2); hold on; plot(tt, dx_2);
% plot(tt, dx_2+x_std(2,:), tt, dx_2-x_std(2,:), Color='r', linestyle='--'); 
% ylabel('$\delta \dot{\hat{x}}(t)$ [m/s]', fontsize=13);
% 
% dx_3 = x_k(3,:)-x_e(3,:);
% subplot(4, 1, 3); hold on; plot(tt, dx_3);
% plot(tt, dx_3+x_std(3,:), tt, dx_3-x_std(3,:), Color='r', linestyle='--'); 
% ylabel('$\delta \hat{\theta}(t)$ [rad]', fontsize=13);
% 
% dx_4 = x_k(4,:)-x_e(4,:);
% subplot(4, 1, 4); hold on; plot(tt, dx_4);
% plot(tt,dx_4+x_std(4,:), tt, dx_4-x_std(4,:), Color='r', linestyle='--'); 
% ylabel('$\delta \dot{\hat{\theta}}$(t)  [rad/s]', fontsize=13);
% xlabel('Time [s]', fontsize=12); 
% 
% lgd = legend('$\delta x(t)$', '$\sigma(t)$', 'fontsize', 13,  'Orientation', 'horizontal');
% set(lgd, 'Position', [0.72, 0.001, 0.1, 0.08]);
% 
% exportgraphics(gcf, 'Fig_errors_1.png', 'Resolution', 300); % 300 DPI for high resolution

%% Control vs. time
fprintf('\nTotal energy: %.2f\n', sum(abs(u_k(1,:)))*dt);
figure; 
sgtitle('Control vs. time', fontsize=14); 

subplot(2, 1, 1); hold on; 
% plot(tt, x_k(3,:), linewidth=2.5); 
plot(tt, u_k(1,:), linewidth=2.5);
ylabel('$u(t) \ [m/s^2]$', fontsize=13);

subplot(2, 1, 2); hold on; 
% plot(tt, x_k(6,:), linewidth=2.5); 
plot(tt, u_k(2,:), linewidth=2.5); 
ylabel('$\dot{u} \ [m/s^3]$', fontsize=13);
xlabel('Time [s]', fontsize=13); 

lgd = legend('$x(t)$', '$u(t)$', 'fontsize', 13, 'Orientation', 'horizontal');
set(lgd, 'Position', [0.75, 0.01, 0.1, 0.08]);

% exportgraphics(gcf, 'Fig_control_1.png', 'Resolution', 300); % 300 DPI for high resolution

%% Control command
% figure; plot(tt, u_sys(x_e), linewidth=2.5)
% figure; plot(tt, w_r(tt), linewidth=2.5 );

%% JUNKYARD

% x_e(:,k+1) = A*x_e(:,k) + B*u_k(k);           % Linear predictor (x'=[A-BK]x)
% P_k = A*P_k*A' + Q;                           % Error state covariance

% Kalman update
% K_k = P_k*C_k'*(inv(C_k*P_k*C_k' + C_k*R*C_k'));  % Kalman gain       
% x_e(:,k+1) = x_e(:,k) + K_k*y_res;        % State vector update
% P_k = (eye(size(x_e,1)) - K_k*C_k) * P_k; % Stace covariance update

% Numerical solvers
% f_xk = x_k(:,k) + f_x( x_k(:,k), u_k(k) )*dt; % State transition function (consider RK-45)
% x_k(:,k+1) = f_xk + w_k();                    % True states
% x_k(:,k+1) = f_Eul(x_k(:,k), u_k(k), dt);

% [t,x] = ode45(@(t,x) f_x(x, u_sys(x)), tt, x_0);  % Deterministic solution

% u = @(x) -K*( x - x_ref + w_r(x) );   % Control law
% u_sat = @(x) max(-u_bnd, min(u_bnd, u(x)));  % Apply saturation to control input
