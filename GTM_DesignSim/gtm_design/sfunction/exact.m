function [sys, x0, str, ts] = exact(t,x,u,flag)

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
sizes.NumInputs      = 3+16+1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

function sys = mdlOutputs(~,~,u)
%      variable     index
% u = [Cld Cmd Cnd    1  2  3 [rad/s^2]
%      U   V   W     4  5  6 [m/s]
%      P   Q   R     7  8  9 [rad/s]
%      q0  q1  q2 q3 10 11 12 13 
%      xi  yi  zi    14 15 16 [m]
%      VT  alp bet   17 18 19 [m/s] [rad] [rad]
%      eta]         20

Cld = u(1); % desired rolling moment coefficient
Cmd = u(2); % desired pitching moment coefficient
Cnd = u(3); % desired yawing moment coefficient

r2d = 180/pi; d2r = pi/180;

P  = u(7);  Q   = u(8);  R   = u(9);
P_deg    =    P*r2d;
Q_deg    =    Q*r2d;
R_deg    =    R*r2d;
VT = u(17); alp = u(18); bet = u(19); % [m/s] [rad]
bet_deg = bet*r2d;
eta = u(20);

[c,b,S,~,~,~]=f18spec(0);
ce = 0.5*c;   % extension wing chord length [m]
be = 0.8*b;   % extension wing span [m]

global ALPHA_BREAK AEROTABLE
aeroco = interp1(ALPHA_BREAK,AEROTABLE,max(min(alp*r2d,90),-14));
% (1) cy_b   ( 8) croll_b   (15) cn_b   (22) cd0    (26) clift0    (30) cm0
% (2) cy_p   ( 9) croll_p   (16) cn_p   (23) cd_q   (27) clift_q   (31) cm_q
% (3) cy_r   (10) croll_r   (17) cn_r   (24) cd_del (28) clift_del (32) cm_del
% (4) cy_da  (11) croll_da  (18) cn_da  (25) cd_der (29) clift_der (33) cm_der
% (5) cy_del (12) croll_del (19) cn_del
% (6) cy_der (13) croll_der (20) cn_der
% (7) cy_dr  (14) croll_dr  (21) cn_dr

F1 =  (aeroco(8)*bet_deg...
      +aeroco(9)*(b/(2*VT))*P_deg...
      +aeroco(10)*(b/(2*VT))*R_deg)... 
      *(S+ce*be*eta)/S*(b+be*eta)/b;
F2 =  (aeroco(30)...
      +aeroco(31)*(c/(2*VT))*Q_deg)...
      *(S+ce*be*eta)/S*(b+be*eta*ce/c)/(b+be*eta);
F3 =  (aeroco(15)*bet_deg...
      +aeroco(16)*(b/(2*VT))*P_deg...
      +aeroco(17)*(b/(2*VT))*R_deg)...
      *(1+ce*be*eta/S)*(b+be*eta)/b;

F = [F1;F2;F3];
  
G = [aeroco(11) aeroco(12)+aeroco(13) aeroco(14);
     0          aeroco(32)+aeroco(33) 0;
     aeroco(18) aeroco(19)+aeroco(20) aeroco(21)];

del_deg = G\([Cld;Cmd;Cnd]-F);

dela = del_deg(1)*d2r; % [rad]
dele = del_deg(2)*d2r; % [rad]
delr = del_deg(3)*d2r; % [rad]

sys = [dela;dele;delr]; % [rad]