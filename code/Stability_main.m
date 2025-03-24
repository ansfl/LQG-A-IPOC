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

% Global execution parameters (equal for both)
pos = 1;                            % Upright position (pos=-1 for bottom)
dt = 0.005; t_f = 15; tt = 0:dt:t_f;
t_go = 1 - tt/tt(end);              % Time normalization

[m_s, M_s, L_s, ~, ~] = sys_params();
[std_w, std_v] = deal(0.0005, 0.0005);
Q_func = @(n_dim) diag(ones(n_dim,1));
clc; close all; 

%% ---------------- Execution parameters ----------------- %
rho = 1;                    % Update : prediction ratio
arr_x = -10:0.5:10; len_x = length(arr_x);
arr_a = -3:0.5:3; len_a = length(arr_a);
[x_dim, a_dim] = meshgrid(arr_a, arr_x);
f_col = [0.8 0.4 0.1]; f_def = [0, 0.4470, 0.7410];

% Visualization parameters
l_s = 20; l_w = 5; 
[v_1, v_2] = deal(-160, 40); 
[z_1, z_2, z_3, z_4] = deal(1000, 80, 400, 100);
[l_1, l_2, l_3, l_4] = deal(l_s*2., l_s/5, l_s*2., l_s/2);

%% ------------------- IPoC Execution (4d) ------------------- %%

[A, B] = IPoC_Linearize( pos ); n_dim = size(A, 1);
Q_lqr = Q_func(n_dim); R_lqr = diag(1/10);
K = lqr(A, B, Q_lqr, R_lqr);
Q = diag( rand(n_dim,1) )*std_w;             % Process noise covariance
R = diag( rand(n_dim,1) )*std_v;             % Measure noise covariance

[Sx_1, Sa_1] = deal(zeros(len_x, len_a), zeros(len_x, len_a));
[Su_1, Ss_1] = deal(zeros(len_x, len_a), zeros(len_x, len_a));

for i=1:len_x
    for j=1:len_a
        IPoC_Init;                             % Initialize system
        x_0   = [0; arr_x(i); pi; arr_a(j)]; % Set initial conditions
        x_k = x_std; x_k(:,1) = x_0;
        x_e = x_std; x_e(:,1) = x_0;
        IPoC_Solve;                            % Solution loop

        % Extract position and angle residuals
        [Sx_1(i,j),Sa_1(i,j)] = deal(abs(x_k(1,end)),abs(x_k(n_dim/2+1,end))); 

        % Extract control expenditure and saturation
        u_tot = trapz(tt, abs(u_k));        
        u_sat = sum( abs(u_k) >= 0.95*u_bnd )/length(u_k)*100;
        [Su_1(i,j),Ss_1(i,j)] = deal(u_tot,u_sat);
    end
    fprintf('# %d\n', i)
end

%% ---------------- Subplots visualization ---------------- %
figure(1);
subplot(2, 2, 1);
contour3(x_dim, a_dim, Sx_1, 'k', 'LineWidth', 0.5, 'LevelStep', l_1); hold on;
contour(x_dim, a_dim, Sx_1, [20 25], 'color', f_def, 'LineWidth', l_w);
xlabel('$\dot{\theta}_0$', fontsize=15);ylabel('$\dot{x}_0$', fontsize=15);zlabel('$| \  x(T) \ |$', fontsize=15);
zlim([0 z_1]); view(v_1, v_2); hold off;

subplot(2, 2, 2);
contour3(x_dim, a_dim, Sa_1, 'k', 'LineWidth', 0.5, 'LevelStep', l_2); hold on;
contour(x_dim, a_dim, Sa_1, [pi-1 pi+1], 'color', f_def, 'LineWidth', l_w);
xlabel('$\dot{\theta}_0$', fontsize=15);ylabel('$\dot{x}_0$', fontsize=15);zlabel('$| \  \theta(T) \ |$', fontsize=15);
zlim([0 z_2]); view(v_1, v_2); hold off;

subplot(2, 2, 3);
contour3(x_dim, a_dim, Su_1, 'k', 'LineWidth', 0.5, 'LevelStep', l_3); hold on;
contour(x_dim, a_dim, Su_1, [0 50], 'color', f_def, 'LineWidth', l_w);
xlabel('$\dot{\theta}_0$', fontsize=15);ylabel('$\dot{x}_0$', fontsize=15);zlabel('$U_{tot}$', fontsize=15);
zlim([0 z_3]); view(v_1, v_2); hold off;

subplot(2, 2, 4);
contour3(x_dim, a_dim, Ss_1, 'k', 'LineWidth', 0.5, 'LevelStep', l_4); hold on;
contour(x_dim, a_dim, Ss_1, [0 1], 'color', f_def, 'LineWidth', l_w);
xlabel('$\dot{\theta}_0$', fontsize=15);ylabel('$\dot{x}_0$', fontsize=15);zlabel('$u_{sat}$', fontsize=15);
zlim([0 z_4]); view(v_1, v_2); hold off;


%% ------------------- A-IPoC Execution (6d) ------------------- %%

[A, B] = A_IPoC_Linearize( pos ); n_dim = size(A, 1);
Q_lqr = Q_func(n_dim); R_lqr = diag([1 1])/10;
K = lqr(A, B, Q_lqr, R_lqr);
Q = diag( rand(n_dim,1) )*std_w;             % Process noise covariance
R = diag( rand(n_dim,1) )*std_v;             % Measure noise covariance

[Sx_2, Sa_2] = deal(zeros(len_x, len_a), zeros(len_x, len_a));
[Su_2, Ss_2] = deal(zeros(len_x, len_a), zeros(len_x, len_a));

for i=1:len_x
    for j=1:len_a
        A_IPoC_Init;                             % Initialize system
        x_0   = [0; arr_x(i); 0; pi; arr_a(j); 0]; % Set initial conditions
        x_k = x_std; x_k(:,1) = x_0;
        x_e = x_std; x_e(:,1) = x_0;
        A_IPoC_Solve;                            % Solution loop

        % Extract position and angle residuals
        [Sx_2(i,j),Sa_2(i,j)] = deal(abs(x_k(1,end)),abs(x_k(n_dim/2+1,end))); 

        % Extract control expenditure and saturation
        u_tot = trapz(tt, abs(u_k(1,:)));        
        u_sat = sum( abs(u_k(1,:)) >= 0.95*u_bnd )/length(u_k)*100;
        [Su_2(i,j),Ss_2(i,j)] = deal(u_tot,u_sat);
    end
    fprintf('# %d\n', i)
end

%% ---------------- Subplots visualization ---------------- %
figure(2);

subplot(2, 2, 1);
contour3(x_dim, a_dim, Sx_2, 'k', 'LineWidth', 0.5, 'LevelStep', l_1); hold on;
contour(x_dim, a_dim, Sx_2, [20 25], 'color', f_col, 'LineWidth', l_w);
xlabel('$\dot{\theta}_0$', fontsize=15);ylabel('$\dot{x}_0$', fontsize=15);zlabel('$| \  x(T) \ |$', fontsize=15);
zlim([0 z_1]); view(v_1, v_2); hold off;

subplot(2, 2, 2);
contour3(x_dim, a_dim, Sa_2, 'k', 'LineWidth', 0.5, 'LevelStep', l_2); hold on;
contour(x_dim, a_dim, Sa_2, [pi-1 pi+1], 'color', f_col, 'LineWidth', l_w);
xlabel('$\dot{\theta}_0$', fontsize=15);ylabel('$\dot{x}_0$', fontsize=15);zlabel('$| \  \theta(T) \ |$', fontsize=15);
zlim([0 z_2]); view(v_1, v_2); hold off;

subplot(2, 2, 3);
contour3(x_dim, a_dim, Su_2, 'k', 'LineWidth', 0.5, 'LevelStep', l_3); hold on;
contour(x_dim, a_dim, Su_2, [0 50], 'color', f_col, 'LineWidth', l_w);
xlabel('$\dot{\theta}_0$', fontsize=15);ylabel('$\dot{x}_0$', fontsize=15);zlabel('$U_{tot}$', fontsize=15);
zlim([0 z_3]); view(v_1, v_2); hold off;

subplot(2, 2, 4);
contour3(x_dim, a_dim, Ss_2, 'k', 'LineWidth', 0.5, 'LevelStep', l_4); hold on;
contour(x_dim, a_dim, Ss_2, [0 10], 'color', f_col, 'LineWidth', l_w);
xlabel('$\dot{\theta}_0$', fontsize=15);ylabel('$\dot{x}_0$', fontsize=15);zlabel('$u_{sat}$', fontsize=15);
zlim([0 z_4]); view(v_1, v_2); hold off;


%% 4 Different figures
% close all;
% l_s = 20; l_w = 5; [v_1, v_2] = deal(-150, 40);
% 
% figure;
% contour3(x_dim, a_dim, S_x, 'k', 'LineWidth', 0.5, 'LevelStep', l_s); hold on;
% contour(x_dim, a_dim, S_x, [20 25], 'color', f_def, 'LineWidth', l_w);
% xlabel('$\dot{\theta}_0$', fontsize=15);
% ylabel('$\dot{x}_0$', fontsize=15);
% zlabel('$| \  x(T) \ |$', fontsize=15);
% view(v_1, v_2); hold off;
% 
% figure;
% contour3(x_dim, a_dim, S_a, 'k', 'LineWidth', 0.5, 'LevelStep', l_s/8); hold on;
% contour(x_dim, a_dim, S_a, [pi-1 pi+1], 'color', f_def, 'LineWidth', l_w);
% xlabel('$\dot{\theta}_0$', fontsize=15);
% ylabel('$\dot{x}_0$', fontsize=15);
% zlabel('$| \  \theta(T) \ |$', fontsize=15);
% view(v_1, v_2); hold off;
% 
% figure;
% contour3(x_dim, a_dim, S_u, 'k', 'LineWidth', 0.5, 'LevelStep', l_s); hold on;
% contour(x_dim, a_dim, S_u, [0 25], 'color', f_def, 'LineWidth', l_w);
% xlabel('$\dot{\theta}_0$', fontsize=15);
% ylabel('$\dot{x}_0$', fontsize=15);
% zlabel('$ U_{tot}(T) $', fontsize=15);
% view(v_1, v_2); hold off;
% 
% figure;
% contour3(x_dim, a_dim, S_s, 'k', 'LineWidth', 0.5, 'LevelStep', l_s/8); hold on;
% contour(x_dim, a_dim, S_s, [0 1], 'color', f_def, 'LineWidth', l_w);
% xlabel('$\dot{\theta}_0$', fontsize=15);
% ylabel('$\dot{x}_0$', fontsize=15);
% zlabel('$ u_{sat}(T) $', fontsize=15);
% view(v_1, v_2); hold off;