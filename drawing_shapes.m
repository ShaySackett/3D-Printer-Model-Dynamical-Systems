%% Q1.4 - Drawing shapes with the decoupled XY model
%% constant variables
% motor constants
r_m = 0.011; L_m = 0.01; R_m = 90; J_m = 8e-6; K_m = 0.07;

% friction constants
mu_r = 0.003; mu_lx = 0.002; mu_ly = 0.001;

% mass
m = 0.06; 

% print plate area
dx = 0.29; dy = 0.29;

% center of print area
cx = dx / 2; cy = dy / 2;

%% initiate simulation
T = linspace(0, 2 * pi, 325)';
u_l = [-zeros(65, 1)', 0.5* cos(T(1:100)/1.2)', 0.4*cos(T(1:100)/0.9)', -ones(60, 1)'];
u_r = [-ones(65, 1)', 0.5*sin(T(1:100)/1.2)', 0.4*sin(T(1:100)/0.9)', -zeros(60, 1)'];
x0 = [cx; 0; 0];
y0 = [cy; 0; 0];

%% state space
D  = 2*J_m + m*r_m^2;
a_x = -2*r_m^2*(4*mu_r + mu_lx)/D;   % friction damps
a_y = -2*r_m^2*(2*mu_r + mu_ly)/D;   % mu_ly here
bT  =  r_m*K_m/D;                    % torque coupling
cE  = -2*K_m/(r_m*L_m);              % back-EMF (theta_l+theta_r = 2x/r_m)
dE  = -R_m/L_m;                      % stable electrical pole

Ax = [0 1 0; 0 a_x bT; 0 cE dE];    Bx = [0;0;1/L_m];
Ay = [0 1 0; 0 a_y bT; 0 cE dE];    By = [0;0;1/L_m];
Cx = [1 0 0];
Cy = [1 0 0];

u_x = u_l + u_r;
u_y = u_l - u_r;

%% transfer function
% Gx = Cx * (inv(s * eye(3) - Ax) \ Bx);
% Gy = Cy * (inv(s * eye(3) - Ay) \ By);

sysX = ss(Ax, Bx, Cx, 0);
X = lsim(sysX, u_x, T, x0);

sysY = ss(Ay, By, Cy, 0);
Y = lsim(sysY, u_y, T, y0);

%% plot simulated trajectories
% plotting X and Y of each other
figure;
plot(X, Y, 'LineWidth', 1.5);
grid on;
axis equal;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Simulated Trajectory');
