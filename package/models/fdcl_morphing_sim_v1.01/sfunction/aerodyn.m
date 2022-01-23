function [sys,x0,str,ts] = aerodyn(t,x,u,flag)

switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 3
        sys=mdlOutputs(t,x,u);
    case {1,2,4,9}
        sys=[];
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes=simsizes;
sizes.NumContStates=0;
sizes.NumDiscStates=0;
sizes.NumOutputs=6;
sizes.NumInputs=3+4+13+2;
sizes.DirFeedthrough=1;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0=[];
str=[];
ts=[0 0];

function sys = mdlOutputs(~,~,u)

global SPEC
S    = SPEC.S;
cbar = SPEC.cbar;
b    = SPEC.b;
Tmax = SPEC.Tmax;

v_w  = u(1:3);
delt = u(4);
dela = u(5);
dele = u(6);
delr = u(7);
eta  = u(21:22);

[~,p_cg] = mass_dist(eta);
x_cg = p_cg(1);
z_cg = p_cg(3); 

state13=u(8:20); % state vector
U   = state13(1);  V   = state13(2);  W   = state13(3);
P   = state13(4);  Q   = state13(5);  R   = state13(6);
q_0 = state13(7);  q_1 = state13(8);  q_2 = state13(9);  q_3 = state13(10);
p_N = state13(11); p_E = state13(12); p_D = state13(13); h   = -p_D;

v = [U;V;W];
q = [q_0;q_1;q_2;q_3];

v_rel = v - quat2dcm(q.')*v_w;
U_rel = v_rel(1); V_rel = v_rel(2); W_rel = v_rel(3);

VT  = norm(v_rel);         % true airspeed [m/s]
alp = atan2(W_rel,U_rel);  % angle of attack [rad]
bet = asin(V_rel/VT);      % sideslip angle [rad]

[~,~,~,rho] = atmosisa(h); % air density [kg/m^3]
qbar = 0.5*rho*VT^2; % dynamic pressure [N/m^2]

[CX,CY,CZ,Cl,Cm,Cn] = aero_model(alp,bet,VT,h,eta,P,Q,R,dela,dele,delr);

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

sys=[F_A+F_T;M_A+M_T];