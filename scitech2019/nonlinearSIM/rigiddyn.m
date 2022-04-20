function [sys,x0,str,ts]=rigiddyn(t,x,u,flag,state130)

switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes(state130);
    case 1
        sys=mdlDerivatives(t,x,u);
    case 3
        sys=mdlOutputs(t,x,u);
    case {2,4,9}
        sys=[];
end

function [sys,x0,str,ts]=mdlInitializeSizes(state130)
sizes=simsizes;
sizes.NumContStates=13;
sizes.NumDiscStates=0;
sizes.NumOutputs=13;
sizes.NumInputs=3+3+2;
sizes.DirFeedthrough=0;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0 = state130;
str=[];
ts=[0 0];

function sys = mdlDerivatives(~,x,u)

global SPEC
mass = SPEC.mass;

v     = x(1:3);
omega = x(4:6);
q     = x(7:10);
F = u(1:3);
M = u(4:6);
eta = u(7:8);
[J,~]=mass_dist(eta);

% force equation
dv = F/mass-cross(omega,v);

% moment equation
domega = J\(M-cross(omega,J*omega));

% kinematic equation
dq = 0.5*[-dot(omega,q(2:4));omega*q(1)-cross(omega,q(2:4))];

% navigation equation
dp = (quat2dcm(q.').')*v;

sys = [dv;domega;dq;dp];

function sys = mdlOutputs(~,x,~)
x(7:10) = quatnormalize(x(7:10).').';

x(2) = 0; % V
x(4) = 0; % P
x(6) = 0; % R
[~,th,~] = quat2angle(x(7:10).');
x(7:10) = angle2quat(0,th,0);
x(12) = 0;

sys=x;