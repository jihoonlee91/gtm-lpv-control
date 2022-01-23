global g SPEC

g = 9.80665; % [m/s^2]

% mass and geometric property

SPEC.mass = 10; % mass [kg]
SPEC.S = 0.84; % reference area (nominal planform area) [m^2]
SPEC.cbar = 0.288; % longitudinal reference length (nominal mean aerodynamic chord) [m]
SPEC.b = 3; % lateral reference length (nominal span) [m]

% control surface limit

SPEC.dela_ll = -.5; % aileron lower limit [rad]
SPEC.dela_ul = .5; % aileron upper limit [rad]
SPEC.dele_ll = deg2rad(-10); % elevator lower limit [rad]
SPEC.dele_ul = deg2rad(10); % elevator upper limit [rad]
SPEC.delr_ll = -.5; % rudder lower limit [rad]
SPEC.delr_ul = .5; % rudder upper limit [rad]

% thruster
SPEC.Tmax = 50; % maximum thrust [N]
zeta = 1;
omega_n = 20; % s^-1