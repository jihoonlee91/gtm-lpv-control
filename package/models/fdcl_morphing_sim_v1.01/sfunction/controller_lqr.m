function [sys,x0,str,ts]=controller_lqr(t,x,u,flag,param)

switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 1
        sys=mdlDerivatives(t,x,u);
    case 3
        sys=mdlOutputs(t,x,u,param);
    case {2,4,9}
        sys=[];
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes=simsizes;
sizes.NumContStates=2+2;
sizes.NumDiscStates=0;
sizes.NumOutputs=4;
sizes.NumInputs=13+3+2+4;
sizes.DirFeedthrough=1;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0 = zeros(4,1);
str=[];
ts=[0 0];

function sys = mdlDerivatives(~,~,u)
VT = u(14);
bet = u(16);
UVWi = quat2dcm(u(7:10).').'*u(1:3); % velocity wrt i frame in i coordinate [Ui Vi Wi]
gamma = -atan2(UVWi(3),sqrt(sum(UVWi(1:2).^2))); % vertical flight path angle
[~,~,ph] = quat2angle(u(7:10).'); % roll angle
sys = [VT-u(19);gamma-u(20);bet-u(21);ph-u(22)]; % command tracking error

function sys = mdlOutputs(~,x,u,param)
Q = u(5);
P = u(4);
R = u(6);
VT = u(14);
alp = u(15);
bet = u(16);
UVWi = quat2dcm(u(7:10).').'*u(1:3); % velocity wrt i frame in i coordinate [Ui Vi Wi]
gamma = -atan2(UVWi(3),sqrt(sum(UVWi(1:2).^2))); % vertical flight path angle
[~,~,ph] = quat2angle(u(7:10).'); % roll angle
xlon = [VT;alp;Q;gamma;x(1:2)]-[param.x_trim_lon;zeros(2,1)]; % trimmed longitudinal state
xlat = [bet;P;R;ph;x(3:4)]-[param.x_trim_lat;zeros(2,1)]; % trimmed lateral state
ctrl_fb = -[param.Klon*xlon;param.Klat*xlat]; % trimmed control
sys = ctrl_fb([1 3 2 4])+param.u_trim; % full control