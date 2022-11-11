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
sizes.NumInputs=3+3+1;
sizes.DirFeedthrough=0;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0 = state130;
str=[];
ts=[0 0];

function sys = mdlDerivatives(~,x,u)
% x = [U V W P Q R q0 q1 q2 q3 xi yi zi]
% u = [X Y Z l m n eta]
g = 9.8;
vel    = x(1:3);                                                           % velocity wrt i frame in b coordinate [U V W]
rate   = x(4:6);                                                           % angular velocity wrt i frame in b coordinate [P Q R]
q   = x(7:10);                                                             % attitude quaternion [q0 q1 q2 q3]
force = u(1:3);
moment = u(4:6);
eta = u(7);
[~,~,~,J,mass,~]=f18spec(eta);

% force equation
veldot = force/mass-cross(rate,vel);

% moment equation
ratedot = J\(moment-cross(rate,J*rate));

% kinematic equation
qdot = 0.5*[-dot(rate,q(2:4));rate*q(1)-cross(rate,q(2:4))];

% navigation equation
posdot = (quat2dcm(q.').')*vel;

sys = [veldot;ratedot;qdot;posdot];

function sys = mdlOutputs(~,x,~)
x(7:10)=quatnormalize(x(7:10).').';                                        % quaternion normalization
sys=x;