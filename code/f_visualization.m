function Visualize(x_t)
% ------------------------ Description ------------------------ %
%   Input  : state vector (x_k)                                 %
%   Output : Graphics at time step k                            %
%   Important : Update x(t) when states are changed !!          %
% -------------------------- Content -------------------------- %

[m, M, L, ~, ~] = sys_params();
x = x_t(1);
n_th = size(x_t,1)/2+1;     % NOTE: Extract \theta state !
th = x_t(n_th);

% Cart graphics 
W = 1*sqrt(M/5);  % cart width
H = .5*sqrt(M/5); % cart height
wr = .2; % wheel radius
mr = .25*sqrt(m); % mass radius

% positions
y = wr/2+H/2;
w1x = x-.75*W/2;    w1y = 0; 
w2x = x+.75*W/2-wr; w2y = 0;

L = 2.5*L; % GRAPHICS Improve
px = x + L*sin(th); py = y - L*cos(th); % Pole

plot([-10 10], [0 0], 'k--', 'LineWidth', 3.); hold on

% Cart
rectangle('Position', [x-W/2,y-H/2,W,H], 'lineWidth', 1.85, 'Curvature', .1, 'FaceColor', [0 0.7 0.8])

% Pendulum wheels
rectangle('Position', [w1x,w1y,wr,wr], 'Curvature', 1, 'EdgeColor', 'k', 'lineWidth', 3.5) % 'FaceColor', 'k', 
rectangle('Position', [w2x,w2y,wr,wr], 'Curvature', 1, 'EdgeColor', 'k', 'lineWidth', 3.5)
% Pendulum pole
c_pole = wr/3;
rectangle('Position', [x-c_pole, y-c_pole, c_pole*2.2, c_pole*2.2], 'Curvature', 1, 'FaceColor', [0 0 0])
plot([x px], [y py], 'k', 'LineWidth', 3.5)
% Pendulum bob
rectangle('Position', [px-mr/2,py-mr/2,mr,mr], 'Curvature', 1, 'FaceColor', [1 0.5 0], 'lineWidth', 2.5)

xlim([-5 5]); 
ylim([-2 3]);
set(gcf,'Position',[2000 300 1000 400])
grid on; drawnow
hold off