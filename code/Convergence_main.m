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
pos = 1;                    % Upright position (pos=-1 for bottom)
dt = 0.005; t_f = 15; tt = 0:dt:t_f;
t_go = 1 - tt/tt(end);              % Time normalization

[m_s, M_s, L_s, ~, ~] = sys_params();
[std_w, std_v] = deal(0.0005, 0.0005);
Q_func = @(n_dim) diag(ones(n_dim,1));

Fig_1_IPoC = zeros([4, 2, length(tt)]);           % Memory allocation
Fig_1_A_IPoC = zeros([4, 2, length(tt)]);         % Memory allocation

Fig_2_IPoC = zeros([4, 2, length(tt)]);           % Memory allocation
Fig_2_A_IPoC = zeros([4, 2, length(tt)]);         % Memory allocation

%% ------------- One time initialization (above) ------------- %%





%% ------------------- Start of Iteration ------------------- %%
clc; close all;
rho = 1;                    % Update : prediction ratio

% Execute IPoC (4d)
[A, B] = IPoC_Linearize( pos ); n_dim = size(A, 1);
Q_lqr = Q_func(n_dim); R_lqr = diag(1/10);
K = lqr(A, B, Q_lqr, R_lqr);
Q = diag( rand(n_dim,1) )*std_w;             % Process noise covariance
R = diag( rand(n_dim,1) )*std_v;             % Measure noise covariance
IPoC_Init;                                   % Initial conditions
IPoC_Solve;                                  % Solution loop
X_IPoC = x_e;                                % Error state vector (IPoC)
f_error_metrics(x_k-x_ref, n_dim, tt, 'IPoC'); % Calulcate error metrics

% Execute A-IPoC (6d)
[A, B] = A_IPoC_Linearize( pos ); n_dim = size(A, 1);
Q_lqr = Q_func(n_dim); R_lqr = diag([1 1])/10;
K = lqr(A, B, Q_lqr, R_lqr);                 % [eig(A) eig(A-B*K)] <-- Open-loop vs closed-loop eigenvalues 
Q = diag( rand(n_dim,1) )*std_w;             % Process noise covariance
R = diag( rand(n_dim,1) )*std_v;             % Measure noise covariance
A_IPoC_Init;                                 % Initial conditions
A_IPoC_Solve;                                % Solution loop
X_A_IPoC = x_e;                              % Error state vector (A-IPoC)
f_error_metrics(x_k-x_ref, n_dim, tt, 'A-IPoC'); % Calulcate error metrics

% Analysis & Visualization

l_w = 2.5; f_col = [0.8 0.4 0.1];
figure; 
subplot(4, 2, 1); hold on; 
plot(tt, X_IPoC(1,:)); plot(tt, X_A_IPoC(1,:), color=f_col, LineWidth=l_w);
ylabel('${x}(t)$', fontsize=13);

subplot(4, 2, 2); hold on;
plot(tt, X_IPoC(3,:)); plot(tt, X_A_IPoC(4,:), color=f_col, LineWidth=l_w);
ylabel('$\theta(t)$', fontsize=14);
xlabel('Time [s]', fontsize=13);

lgd = legend('IPoC', 'A-IPoC', 'fontsize', 13, 'Orientation', 'horizontal');
set(lgd, 'Position', [0.66, 0.01, 0.1, 0.08]);

%
figure; subplot(2, 2, 1);
hold on; % grid on;
k_s = 2; l_w = 2.;

X_IPoC_n   = [X_IPoC(1,:)/X_IPoC(1,1); (X_IPoC(3,:)-pi)/(X_IPoC(3,1)-pi)];
X_A_IPoC_n = [X_A_IPoC(1,:)/X_A_IPoC(1,1); (X_A_IPoC(4,:)-pi)/(X_A_IPoC(4,1)-pi)];

plot3(X_IPoC_n(1,:), X_IPoC_n(2,:), t_go, LineWidth=l_w); 
plot3(X_A_IPoC_n(1,:), X_A_IPoC_n(2,:), t_go, LineWidth=l_w, color=f_col); 

% plot3(X_IPoC(1,1:k_s:end)/X_IPoC(1,1), (X_IPoC(3,1:k_s:end)-pi)/(X_IPoC(3,1)-pi), t_go(1:k_s:end), LineWidth=l_w); 
% plot3(X_A_IPoC(1,1:k_s:end)/X_A_IPoC(1,1), (X_A_IPoC(4,1:k_s:end)-pi)/(X_A_IPoC(4,1)-pi), t_go(1:k_s:end), LineWidth=l_w, color=f_col); 

sz = 80;
scatter3(1,1,1, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0 .75 .75]);
scatter3(0,0,0, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [1. 0. 0.]);

xlabel('$\bar{x}$', fontsize=14);
ylabel('$\bar{\theta}$', fontsize=14);
zlabel('$\bar{t}_{go}$', fontsize=14);
view(30,30)
% clc; close all


%% Save data for subplots

nn = 1; % UPDATE Iteration number !!!

% Figure #1
[Fig_1_IPoC(nn,1,:) , Fig_1_IPoC(nn,2,:)]    = deal(X_IPoC(1,:), X_IPoC(3,:));
[Fig_1_A_IPoC(nn,1,:), Fig_1_A_IPoC(nn,2,:)] = deal(X_A_IPoC(1,:), X_A_IPoC(4,:));

% Figure #2
[Fig_2_IPoC(nn,1,:) , Fig_2_IPoC(nn,2,:)]    = deal(X_IPoC_n(1,:), X_IPoC_n(2,:));
[Fig_2_A_IPoC(nn,1,:), Fig_2_A_IPoC(nn,2,:)] = deal(X_A_IPoC_n(1,:), X_A_IPoC_n(2,:));

%% Plot subplots

figure; l_w = 2.5; f_col = [0.8 0.4 0.1];

% ------------------------------------------------- %
subplot(4, 2, 1); hold on; 
plot(tt, squeeze(Fig_1_IPoC(1,1,:))); plot(tt, squeeze(Fig_1_A_IPoC(1,1,:)), color=f_col, LineWidth=l_w);
ylabel('$x(t)$', fontsize=14);

subplot(4, 2, 2); hold on;
plot(tt, squeeze(Fig_1_IPoC(1,2,:))); plot(tt, squeeze(Fig_1_A_IPoC(1,2,:)), color=f_col, LineWidth=l_w);
ylabel('$\theta(t)$', fontsize=14);

% ------------------------------------------------- %
subplot(4, 2, 3); hold on;
plot(tt, squeeze(Fig_1_IPoC(2,1,:))); plot(tt, squeeze(Fig_1_A_IPoC(2,1,:)), color=f_col, LineWidth=l_w);
ylabel('$x(t)$', fontsize=14);

subplot(4, 2, 4); hold on;
plot(tt, squeeze(Fig_1_IPoC(2,2,:))); plot(tt, squeeze(Fig_1_A_IPoC(2,2,:)), color=f_col, LineWidth=l_w);
ylabel('$\theta(t)$', fontsize=14);

% ------------------------------------------------- %
subplot(4, 2, 5); hold on;
plot(tt, squeeze(Fig_1_IPoC(3,1,:))); plot(tt, squeeze(Fig_1_A_IPoC(3,1,:)), color=f_col, LineWidth=l_w);
ylabel('$x(t)$', fontsize=14);

subplot(4, 2, 6); hold on;
plot(tt, squeeze(Fig_1_IPoC(3,2,:))); plot(tt, squeeze(Fig_1_A_IPoC(3,2,:)), color=f_col, LineWidth=l_w);
ylabel('$\theta(t)$', fontsize=14);

% ------------------------------------------------- %
subplot(4, 2, 7); hold on;
plot(tt, squeeze(Fig_1_IPoC(4,1,:))); plot(tt, squeeze(Fig_1_A_IPoC(4,1,:)), color=f_col, LineWidth=l_w);
ylabel('$x(t)$', fontsize=14);
xlabel('Time [s]', fontsize=14);

subplot(4, 2, 8); hold on;
plot(tt, squeeze(Fig_1_IPoC(4,2,:))); plot(tt, squeeze(Fig_1_A_IPoC(4,2,:)), color=f_col, LineWidth=l_w);
ylabel('$\theta(t)$', fontsize=14);
xlabel('Time [s]', fontsize=14);

lgd = legend('IPoC', 'A-IPoC', 'fontsize', 13, 'Orientation', 'horizontal');
set(lgd, 'Position', [0.44, 0.01, 0.1, 0.08]);
exportgraphics(gcf, 'Fig_Conv_1.png', 'Resolution', 300); % 300 DPI for high resolution


% 3D Tracking Error Analysis

%%

figure; k_s = 1; l_w = 2.5; sz = 80;
[v_1, v_2] = deal(-27, 10);

% ------------------------------------------------- %
subplot(2, 2, 1); hold on; grid on;
plot3( squeeze(Fig_2_IPoC(1,1,1:k_s:end)), squeeze(Fig_2_IPoC(1,2,1:k_s:end)), t_go(1:k_s:end), LineWidth=l_w); 
plot3( squeeze(Fig_2_A_IPoC(1,1,1:k_s:end)), squeeze(Fig_2_A_IPoC(1,2,1:k_s:end)), t_go(1:k_s:end), LineWidth=l_w, color=f_col); 
scatter3(1,1,1, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0 .75 .75]);
scatter3(0,0,0, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [1. 0. 0.]);
xlabel('$\bar{x}(t)$', fontsize=14);
ylabel('$\bar{\theta}(t)$', fontsize=14);
zlabel('$\bar{t}_{go}(t)$', fontsize=14);
view(v_1, v_2)

% ------------------------------------------------- %
subplot(2, 2, 2); hold on; grid on;
plot3( squeeze(Fig_2_IPoC(2,1,1:k_s:end)), squeeze(Fig_2_IPoC(2,2,1:k_s:end)), t_go(1:k_s:end), LineWidth=l_w); 
plot3( squeeze(Fig_2_A_IPoC(2,1,1:k_s:end)), squeeze(Fig_2_A_IPoC(2,2,1:k_s:end)), t_go(1:k_s:end), LineWidth=l_w, color=f_col); 
scatter3(1,1,1, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0 .75 .75]);
scatter3(0,0,0, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [1. 0. 0.]);
xlabel('$\bar{x}(t)$', fontsize=14);
ylabel('$\bar{\theta}(t)$', fontsize=14);
zlabel('$\bar{t}_{go}(t)$', fontsize=14);
view(v_1, v_2)

% ------------------------------------------------- %
subplot(2, 2, 3); hold on; grid on;
plot3( squeeze(Fig_2_IPoC(3,1,1:k_s:end)), squeeze(Fig_2_IPoC(3,2,1:k_s:end)), t_go(1:k_s:end), LineWidth=l_w); 
plot3( squeeze(Fig_2_A_IPoC(3,1,1:k_s:end)), squeeze(Fig_2_A_IPoC(3,2,1:k_s:end)), t_go(1:k_s:end), LineWidth=l_w, color=f_col); 
scatter3(1,1,1, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0 .75 .75]);
scatter3(0,0,0, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [1. 0. 0.]);
xlabel('$\bar{x}(t)$', fontsize=14);
ylabel('$\bar{\theta}(t)$', fontsize=14);
zlabel('$\bar{t}_{go}(t)$', fontsize=14);
view(v_1, v_2)

% ------------------------------------------------- %
subplot(2, 2, 4); hold on; grid on;
plot3( squeeze(Fig_2_IPoC(4,1,1:k_s:end)), squeeze(Fig_2_IPoC(4,2,1:k_s:end)), t_go(1:k_s:end), LineWidth=l_w); 
plot3( squeeze(Fig_2_A_IPoC(4,1,1:k_s:end)), squeeze(Fig_2_A_IPoC(4,2,1:k_s:end)), t_go(1:k_s:end), LineWidth=l_w, color=f_col); 
scatter3(1,1,1, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0 .75 .75]);
scatter3(0,0,0, sz, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [1. 0. 0.]);
xlabel('$\bar{x}(t)$', fontsize=14);
ylabel('$\bar{\theta}(t)$', fontsize=14);
zlabel('$\bar{t}_{go}(t)$', fontsize=14);
view(v_1, v_2)
exportgraphics(gcf, 'Fig_Conv_2.png', 'Resolution', 300); % 300 DPI for high resolution

% % For clarity: subtract pi from angular state
% X_IPoC(3,:)   = X_IPoC(3,:)-pi; 
% X_A_IPoC(4,:) = X_A_IPoC(4,:)-pi;
% 
% % Normalize trajectory unless I.C. == 0
% if X_IPoC(1,1)   ~= 0, X_IPoC(1,:)   = X_IPoC(1,:)/X_IPoC(1,1); end
% if X_IPoC(3,1)   ~= 0, X_IPoC(3,:)   = X_IPoC(3,:)/X_IPoC(3,1); end
% if X_A_IPoC(1,1) ~= 0, X_A_IPoC(1,:) = X_A_IPoC(1,:)/X_A_IPoC(1,1); end
% if X_A_IPoC(4,1) ~= 0, X_A_IPoC(4,:) = X_A_IPoC(4,:)/X_A_IPoC(4,1); end