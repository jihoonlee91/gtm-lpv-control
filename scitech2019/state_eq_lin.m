% dynamics is identical to that of state_eq.m
% BUT input and output are different from them of state_eq.m

function dx = state_eq_lin(x,u,eta)

global SPEC
mass = SPEC.mass;
S    = SPEC.S;
cbar = SPEC.cbar;
b    = SPEC.b;
Tmax = SPEC.Tmax;

% state and control variables are unfolded here

delt = u(1); % throttle
dele = u(2); % elevator
dela = u(3); % aileron
delr = u(4); % rudder

VT  = x(1);  alp = x(2);  bet = x(3);
P   = x(4);  Q   = x(5);  R   = x(6);
ph  = x(7);  th  = x(8);  ps  = x(9);
p_N = x(10); p_E = x(11); p_D = x(12); h = -p_D;

U = VT*cos(alp)*cos(bet);
V = VT*sin(bet);
W = VT*sin(alp)*cos(bet);

% specifications of the model aircraft are given here

[J,p_ref] = mass_dist(eta);
x_ref = p_ref(1);
z_ref = p_ref(3);

% vectorization

v     = [U;V;W];
omega = [P;Q;R];
q     = angle2quat(ps,th,ph).';

[~,~,~,rho] = atmosisa(h); % air density [kg/m^3]
qbar = 0.5*rho*VT^2; % dynamic pressure [N/m^2]

% aerodynamic force and moment are computed here

[CX,CY,CZ,Cl,Cm,Cn] = aero_model(alp,bet,VT,h,eta,P,Q,R,dela,dele,delr); % body-axis aerodynamic coefficient

X_A = qbar*CX*S; % aerodynamic force along body x-axis
Y_A = qbar*CY*S; % aerodynamic force along body y-axis
Z_A = qbar*CZ*S; % aerodynamic force along body z-axis

l_A = qbar*S*b*Cl    + z_ref*Y_A;    % aerodynamic moment with respect to body x axis
m_A = qbar*S*cbar*Cm + x_ref*Z_A ...
                     - z_ref*X_A;    % aerodynamic moment with respect to body y axis
n_A = qbar*S*b*Cn    - x_ref*Y_A;    % aerodynamic moment with respect to body z axis

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

dv = F/mass-cross(omega,v);
dU = dv(1);
dV = dv(2);
dW = dv(3);
dVT = (U*dU+V*dV+W*dW)/VT;
dalp = (U*dW-W*dU)/(U^2+W^2);
dbet = (VT*dV-V*dVT)/(cos(bet)*VT^2) ; 
dwind = [dVT;dalp;dbet];

domega = pinv(J)*(M-cross(omega,J*omega)); % rotational equations of motion

H = [1 sin(ph)*tan(th) cos(ph)*tan(th);
     0 cos(ph)        -sin(ph);
     0 sin(ph)/cos(th) cos(ph)/cos(th)];
dangle = H*omega; % kinematic equation

dp = quat2dcm(q.').'*v; % navigation equation

% state derivative

dx = [dwind;domega;dangle;dp];