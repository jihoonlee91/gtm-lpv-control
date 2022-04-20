% this function is used only for trim calculation
% this function is NOT used for simulation

function dx = state_eq(x,u,eta)

global SPEC
mass = SPEC.mass;
S    = SPEC.S;
cbar = SPEC.cbar;
b    = SPEC.b;
Tmax = SPEC.Tmax;

% state and control variables are unfolded here

delt = u(1); % throttle
dela = u(2); % aileron
dele = u(3); % elevator
delr = u(4); % rudder

U   = x(1);  V   = x(2);  W   = x(3);               % velocity [m/s]
P   = x(4);  Q   = x(5);  R   = x(6);               % angular velocity [rad/s]
q_0 = x(7);  q_1 = x(8);  q_2 = x(9);  q_3 = x(10); % quaternion
p_N = x(11); p_E = x(12); p_D = x(13); h   = -p_D;  % position [m]

% specifications of the model aircraft are given here

[J,p_cg] = mass_dist(eta);
x_cg = p_cg(1);
z_cg = p_cg(3); 

% vectorization

v     = [U;V;W];
omega = [P;Q;R];
q     = [q_0;q_1;q_2;q_3];

% environment

VT  = norm(v);     % true airspeed [m/s]
alp = atan2(W,U);  % angle of attack [rad]
bet = asin(V/VT);  % sideslip angle [rad]

[~,~,~,rho] = atmosisa(h); % air density [kg/m^3]
qbar = 0.5*rho*VT^2; % dynamic pressure [N/m^2]

% aerodynamic force and moment are computed here

[CX,CY,CZ,Cl,Cm,Cn] = aero_model(alp,bet,VT,h,eta,P,Q,R,dela,dele,delr); % body-axis aerodynamic coefficient

X_A = qbar*CX*S; % aerodynamic force along body x-axis
Y_A = qbar*CY*S; % aerodynamic force along body y-axis
Z_A = qbar*CZ*S; % aerodynamic force along body z-axis

l_A = qbar*S*b*Cl    + z_cg*Y_A;    % aerodynamic moment with respect to body x axis
m_A = qbar*S*cbar*Cm + x_cg*Z_A ...
                     - z_cg*X_A;    % aerodynamic moment with respect to body y axis
n_A = qbar*S*b*Cn    - x_cg*Y_A;    % aerodynamic moment with respect to body z axis

F_A = [X_A;Y_A;Z_A]; % aerodynamic force [N]
M_A = [l_A;m_A;n_A]; % aerodynamic moment [N*m]

% thruster force and moment are computed here

T = Tmax*delt; % thrust [N]
X_T = T; Y_T = 0; Z_T = 0; % thruster force body axes component [N]
l_T = 0; m_T = 0; n_T = 0; % thruster moment body axes component [N*m]

F_T = [X_T;Y_T;Z_T]; % thruster force in body coordinate [N]
M_T = [l_T;m_T;n_T]; % thruster moment in body coordinate [N*m]

% gravitational force and moment are computed here

global g
F_g = quat2dcm(q.')*[0;0;mass*g]; % gravitational force in body coordinate [N]
M_g = zeros(3,1); % gravitational moment in body coordinate [N*m]

% body-axis force and moment

F = F_A + F_T + F_g;
M = M_A + M_T + M_g;

% state equation

dv = F/mass-cross(omega,v); % translational equations of motion
domega = pinv(J)*(M-cross(omega,J*omega)); % rotational equations of motion
dq = 0.5*[-dot(omega,q(2:4));omega*q(1)-cross(omega,q(2:4))]; % kinematic equation
dp = quat2dcm(q.').'*v; % navigation equation

% state derivative

dx = [dv;domega;dq;dp];