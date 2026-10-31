%% Numerical calculation of the PDE-filament problem
% variables
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

% discretization
nozzle_width = 1e-3;                  % nozzle of 1mm [m]
nNodes = L_tot/nozzle_width;          % number of spatial nodes
dx = L_tot/(nNodes - 1);              % spatial step [m]
% x = linspace(0, L_tot, nNodes).';     % spatial grid [m]
dt = dx/v;                            % time step based on print speed [s]

% temperature array
T = T_e * ones(1, nNodes);   % array of ambient temperatures [K]
T(1) = T_n;

%% INITIALIZATION
n_deposits = 0; % number of deposits done by the nozzle
spread = inf;   % initialized spread between Tmax and Tmin
tol = 1;      % degrees tolerance of spread
t = 0;
x = 0;

%% TIME MARCHING
while ~(t >= t_end && spread < tol)
    Tnew = T;

    % new deposit of filament
    if t < t_end 
        n_deposits = n_deposits + 1;
        Tnew(n_deposits) = T_n;
        T(n_deposits)    = T_n;
    end

    % apply fourier law to all deposits
    for i = 1:n_deposits
        % open
        if x < L_tot
            if i == n_deposits  % newest deposit (held at Tn)
                continue
            elseif i == 1       % first deposit
                n_air_sides = 4;
                Txx = 2 * (T(2) - T(1)) / dx^2;
            else                % all other deposits
                n_air_sides = 3;
                Txx = (T(i-1) - 2 * T(i) + T(i+1)) / (dx ^ 2);
            end
        % closed
        else
            n_air_sides = 3;
            % first and last deposit connected
            if i == 1
                Txx = (T(n_deposits) - 2 * T(i) + T(i+1)) / (dx ^ 2);
            elseif i == n_deposits
                Txx = (T(i-1) - 2 * T(i) + T(1)) / (dx ^ 2);
            else
                Txx = (T(i-1) - 2 * T(i) + T(i+1)) / (dx ^ 2);
            end
        end
        % exposed sides
        f_air = n_air_sides * k_air * (T_e - T(i)) / (w * rho * c);
        f_plate = k_plate  * (T_p - T(i)) / (w * rho * c);
        % update deposit
        Tnew(i) = T(i) + dt * (k * Txx + f_air + f_plate);
    end
    
    T = Tnew;
    x = x + dx;
    t = t + dt;
    [minT, maxT] = bounds(T(1:n_deposits));
    spread = maxT - minT;
end

%% DISPLAY RESULTS
fprintf('print done at %.1f s;, uniform (spread<%g K) at %.1f s (%.1f s after print)\n', ...
        t_end, tol, t, t-t_end);
fprintf('final temperatures: %.1f - %.1f degC\n', min(T)-273, max(T)-273);