function [sys, x0, str, ts] = pdControl(t,x,u,flag,th0)

switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes(th0);
    case 1
        sys=mdlDerivatives(t,x,u);
    case 3
        sys=mdlOutputs(t,x,u);
    case {2,4,9}
        sys=[];
end

function [sys,x0,str,ts] = mdlInitializeSizes(th0)
sizes = simsizes;
sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3+3+3;
sizes.NumInputs      = 3+13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [0;th0;0];
str = [];
ts  = [0 0];

function sys = mdlDerivatives(~,~,u)
%      variable     index
% u = [Pc Qc  Rc    1  2  3 [rad/s]
%      U  V   W     4  5  6 [m/s]
%      P  Q   R     7  8  9 [rad/s]
%      q0 q1  q2 q3 10 11 12 13
%      xi yi  zi]   14 15 16 [m]
cmdRate = u(1:3); % [rad/s]
[~,th,ph] = quat2angle(u(10:13).','ZYX');

H = [1 sin(ph)*tan(th) cos(ph)*tan(th);
     0 cos(ph)        -sin(ph);
     0 sin(ph)*sec(th) cos(ph)*sec(th)];
cmdAngledot = H*cmdRate;

sys = cmdAngledot; % [rad/s]

function sys = mdlOutputs(~,x,u)
cmdRate = u(1:3); % [rad/s]
rate = u(7:9); % [rad/s]
% quat_c = angle2quat(x(3),x(2),x(1),'ZYX');
% [ps_c,th_c,ph_c] = quat2angle(quat_c,'ZYX');

cmdAngle = x;
ph_c = cmdAngle(1); % [rad]
th_c = cmdAngle(2); % [rad]
ps_c = cmdAngle(3); % [rad]

[ps,th,ph] = quat2angle(u(10:13).','ZYX');
ps = atan(tan(ps));
H = [1 sin(ph)*tan(th) cos(ph)*tan(th);
     0 cos(ph)        -sin(ph);
     0 sin(ph)*sec(th) cos(ph)*sec(th)];
cmdAngledot = H*cmdRate;
dph_c = cmdAngledot(1); % [rad/s]
dth_c = cmdAngledot(2); % [rad/s]
dps_c = cmdAngledot(3); % [rad/s]

angledot = H*rate;
dph = angledot(1); % [rad/s]
dth = angledot(2); % [rad/s]
dps = angledot(3); % [rad/s]

tsi = 1.5; % [s]
% tsi = 3; % [s]
kd = 9.2/tsi; % [s^-1]
kp = (kd^2)/2;% [s^-2]

ph_e = atan(tan(ph_c-ph));
th_e = atan(tan(th_c-th));
ps_e = atan(tan(ps_c-ps));

% ph_e = 0;
% th_e = 0;
% ps_e = 0;

U1 = kp*ph_e+kd*(dph_c-dph); % [rad/s^2]
U2 = kp*th_e+kd*(dth_c-dth); % [rad/s^2]
U3 = kp*ps_e+kd*(dps_c-dps); % [rad/s^2]

sys = [U1;U2;U3;ph_c;th_c;ps_c;dph_c;dth_c;dps_c]; % [rad/s^2]