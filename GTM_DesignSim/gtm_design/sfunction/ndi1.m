function [sys, x0, str, ts] = ndi1(t,x,u,flag)

switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 3
        sys=mdlOutputs(t,x,u);
    case {1,2,4,9}
        sys=[];
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 3+16+1; % rate command, state, eta
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

function sys = mdlOutputs(~,~,u)
%      variable     index
% u = [U1 U2  U3    1  2  3 [rad/s^2]
%      U  V   W     4  5  6 [m/s]
%      P  Q   R     7  8  9 [rad/s]
%      q0 q1  q2 q3 10 11 12 13 
%      xi yi  zi    14 15 16 [m]
%      VT alp bet   17 18 19 [m/s] [rad] [rad]
%      eta]         20

U1 = u(1);  U2  = u(2);  U3 = u(3); % [rad/s^2]
P  = u(7);  Q   = u(8);  R  = u(9); % [rad/s]
rate = [P;Q;R];
VT = u(17); % [m/s]
zi = u(16); % [m]
h  = -zi; % altitude
eta = u(20);

[~,th,ph] = quat2angle(u(10:13).','ZYX');
H = [1 sin(ph)*tan(th) cos(ph)*tan(th);
     0 cos(ph)        -sin(ph);
     0 sin(ph)*sec(th) cos(ph)*sec(th)];
angledot = H*rate;
dph = angledot(1); % [rad/s]
dth = angledot(2); % [rad/s]
dps = angledot(3); % [rad/s]

dPd =  U1-U3*sin(th)-dth*dps*cos(th); % [rad/s^2]
dQd =  U2*cos(ph)-dph*dth*sin(ph)+U3*sin(ph)*cos(th)+dph*dps*cos(ph)*cos(th)-dth*dps*sin(ph)*sin(th); % [rad/s^2]
dRd = -U2*sin(ph)-dph*dth*cos(ph)+U3*cos(ph)*cos(th)-dph*dps*sin(ph)*cos(th)-dth*dps*cos(ph)*sin(th); % [rad/s^2]

[c,b,S,~,~,~]=f18spec(0);
[~,~,~,J,~,~]=f18spec(eta);
[~,~,~,rho]=atmosisa(h);
q = 0.5*rho*VT^2;

Md = J*[dPd;dQd;dRd]+cross(rate,J*rate); % desired moment
Cld = Md(1)/(q*S*b); % desired rolling moment coefficient
Cmd = Md(2)/(q*S*c); % desired pitching moment coefficient
Cnd = Md(3)/(q*S*b); % desired yawing moment coefficient

sys = [Cld;Cmd;Cnd];