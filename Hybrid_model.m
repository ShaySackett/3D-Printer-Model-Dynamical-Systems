%% Parameters
% Parameters for the filament cooldown
rho = 1240;         % PLA-density [kg/m-3]
c = 1600;           % specific heat capacity [J/kgK]
k = 0.13/(rho*c);   % thermal conductivity of filament [m^2/s]
w = 0.45e-3;        % line width of 0.45mm
A = w^2;            % cross section
k_air = 30;         % convection coefficient [W/m^2K]
k_plate = 300;      % effective coefficient [W/m^2K]
T_n = 200 + 273;    % nozzle temperature [K]
T_p = 60 + 273;     % plate temperature [K]
T_e = 20 + 273;     % environmental temperature [K]
v = 50e-3;          % print speed [m/s]
L_tot = 0.255;      % total length of a 5x3cm elliptical path
t_end = L_tot/v;    % total print time to complete one path

% Parameters for the drawing of shapes
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

% build both systems once, reuse them for the two shapes
sysX = ss(Ax, Bx, Cx, 0);
sysY = ss(Ay, By, Cy, 0);

%% Mealy

in    = 'start';
state = 'stop';
out   = 'red';

while ~strcmp(in, 'break') && ~strcmp(state, 'block')
    display(out);
    in = input('Next input? ', 's');
    switch state
        case 'stop'
            switch in
                case 'push'
                    state = 'go';
                    out   = 'green';
                case 'timeout'
                    state = 'stop';
                    out   = 'red';
            end
        case 'go'
            switch in
                case 'push'      % NOTE: the diagram shows go --push--> stop,
                    state = 'go';    % but the original code keeps state 'go'
                    out   = 'green';
                case 'timeout'
                    state = 'stop';
                    out   = 'red';
            end
    end
end