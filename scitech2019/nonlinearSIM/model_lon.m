function [sys,x0,str,ts]=model_lon(t,x,u,flag)

switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 1
        sys=mdlDerivatives(t,x,u);
    case 3
        sys=mdlOutputs(t,x,u);
    case {2,4,9}
        sys=[];
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes=simsizes;
sizes.NumContStates=5;
sizes.NumDiscStates=0;
sizes.NumOutputs=8;
sizes.NumInputs=4;
sizes.DirFeedthrough=1;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0 = [20;0.072498;0;0.072498;300]; % V_T alpha q theta h
str=[];
ts=[0 0];

function sys = mdlDerivatives(t,x,u)
V_T = x(1);
alp = x(2);
q = x(3);
theta = x(4);
h = x(5);
eta_1 = u(1);
eta_2 = u(2);
dele = u(3);
delt = u(4);

eta = [eta_1;eta_2];

global SPEC
S    = SPEC.S;
cbar = SPEC.cbar;
Tmax = SPEC.Tmax;
mass = SPEC.mass;
g = 9.80665;

[J,~] = mass_dist(eta);
J_y = J(2,2);

[~,~,~,rho] = atmosisa(h); % air density [kg/m^3]
q_bar = 0.5*rho*V_T^2; % dynamic pressure [N/m^2]

[~,~,~,~,Cm,~,CD,CL] = aero_model(alp,0,V_T,h,eta,0,q,0,0,dele,0);

L = q_bar*S*CL; % aerodynamic force along body x-axis
D = q_bar*S*CD; % aerodynamic force along body z-axis
m = q_bar*S*cbar*Cm;

T = Tmax*delt; % thrust [N]

dV_T = (T*cos(alp)-D)/mass-g*sin(theta-alp);
dalp = (-T*sin(alp)-L)/(mass*V_T)+g*cos(theta-alp)/V_T+q;
dq = m/J_y;
dtheta = q;
dh = V_T*sin(theta-alp);

dx = [dV_T;dalp;dq;dtheta;dh];

sys = dx;

function sys = mdlOutputs(t,x,u)
y = zeros(3,1);
y(1) = x(2);
y(2) = x(3);

V_T = x(1);
alp = x(2);
q = x(3);
h = x(5);
eta_1 = u(1);
eta_2 = u(2);
dele = u(3);

eta = [eta_1;eta_2];

global SPEC
S    = SPEC.S;
mass = SPEC.mass;
g = 9.80665;

[~,~,~,rho] = atmosisa(h); % air density [kg/m^3]
q_bar = 0.5*rho*V_T^2; % dynamic pressure [N/m^2]

[~,~,~,~,~,~,CD,CL] = aero_model(alp,0,V_T,h,eta,0,q,0,0,dele,0);

L = q_bar*S*CL; % aerodynamic force along body x-axis
D = q_bar*S*CD; % aerodynamic force along body z-axis

Z =  sin(alp)*(-D)+cos(alp)*(-L);
y(3) = Z/(mass*g);

sys = [x;y];