function [sys, x0, str, ts] = cas(t,x,u,flag)

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


function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1+2;
sizes.NumInputs      = 3+3+16;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(2,1);
str = [];
ts  = [0 0];

function sys = mdlDerivatives(~,~,u)
%      variable     index
% u = [Pc anc ayc   1  2  3
%      ax ay  az    4  5  6
%      U  V   W     7  8  9
%      P  Q   R     10 11 12
%      q0 q1  q2 q3 13 14 15 16
%      xi yi  zi    17 18 19
%      VT alp bet]  20 21 22

anc = u(2); ayc = u(3);                                                                % y-axis accelration command [g]
ay  = u(5); az  = u(6);                                                    % acceleration in b coordinate [g]
an  = -az;                                                                 % normal acceleration [g]

e_an = anc-an;
e_ay = ayc-ay;

sys = [e_an;e_ay];

function sys = mdlOutputs(~,x,u)
g0 = 9.8;                                                                  % gravitational acceleration at sea level [m/s^2]
g  = 9.8;                                                                  % gravitational acceleration [m/s^2]

int_e_an = x(1);
int_e_ay = x(2);

Pc  = deg2rad(u(1));                                                       % roll rate command [deg/s]
P=u(10); % [rad/s]

anc = u(2); ayc = u(3);
ay  = u(5); az  = u(6);                                                    % acceleration in b coordinate [g]
an  = -az;                                                                 % normal acceleration [g]

U  = u(7);  V = u(8);  W = u(9); % [m/s]
zi = u(19); h = -zi;
[~,~,~,rho] = atmosisa(h); % [kg/m^3]

VT = u(20); % [m/s]
[~,th,ph] = quat2angle(u(13:16).','ZYX');

d2r = pi/180; r2d = 180/pi;
ALPHA_BREAK = (-14:4:90).'; % [deg]
alp = u(21); alp_deg = alp*r2d; dalp_deg = 0.5;
alp_deg = max(min(alp_deg,90-0.5*dalp_deg),-14+0.5*dalp_deg);

clift0 = [ -1.03860  -0.880907  -0.551318  -0.216206   0.154461   0.560857 ...
            0.951575  1.24168    1.39786    1.53103    1.64110    1.77724    1.88652 ...
            1.88940   1.80975    1.68556    1.57524    1.46011    1.31884    1.17117 ...
            1.02151   0.858875   0.685334   0.529482   0.378268   0.229349   9.10933E-02 ]';
cy_b = [   -1.71957E-02   -1.71957E-02   -1.77407E-02   -1.82857E-02   -1.84197E-02 ...
           -1.84159E-02   -1.87763E-02   -1.77142E-02   -1.55057E-02   -1.40980E-02 ...	
           -1.28732E-02   -1.26852E-02   -1.25160E-02   -1.35594E-02   -1.40314E-02 ...
           -1.33128E-02   -1.19496E-02   -1.30678E-02   -1.37112E-02   -1.37528E-02 ...
           -1.33426E-02   -1.29076E-02   -1.33076E-02   -1.38320E-02   -1.37811E-02 ...
           -1.31714E-02   -1.27360E-02]';

% CLalp = (interp1(ALPHA_BREAK,clift0,alp_deg+0.5*dalp_deg)-interp1(ALPHA_BREAK,clift0,alp_deg-0.5*dalp_deg))/(dalp_deg*pi/180);
CLalp = 6;
% Cybet = interp1(ALPHA_BREAK,cy_b,alp_deg);
% Cybet = -1.076;
Cybet = 2;

[~,~,S,~,m,~]=f18spec(0);
taun = 2*m/(rho*S*VT*CLalp); % [s]
tauy = 2*m/(rho*S*VT*abs(Cybet)); % [s]
tso = 4.5; % [s]
K2 = 4.6/tso; K4 = 4.6/tso; % [s^-1]
K1 = taun*K2; K3 = tauy*K4;

if abs(U) < eps
    Qc = 0;
    Rc = 0;
else
    Qc = K1*(anc-an)*g0/U + K2*g0*int_e_an/U + (V*P-g*cos(ph)*cos(th))/U;
    Rc = K3*(ayc-ay)*g0/U + K4*g0*int_e_ay/U + (W*P+g*sin(ph)*cos(th))/U;
end

sys = [Pc;Qc;Rc];