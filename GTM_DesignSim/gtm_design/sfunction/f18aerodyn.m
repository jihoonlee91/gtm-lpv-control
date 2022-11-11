function [sys,x0,str,ts] = f18aerodyn(t,x,u,flag)

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
sizes.NumInputs=3+4+13+1;
sizes.DirFeedthrough=1;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0=[];
str=[];
ts=[0 0];

function sys = mdlOutputs(~,~,u)
% u=[Uiw Viw Wiw         1 2 3       [m/s]
%    delt dela dele delr 4 5 6 7     [], [rad]
%    U V W               8 9 10      [m/s]
%    P Q R               11 12 13    [rad/s]
%    q0 q1 q2 q3         14 15 16 17 []
%    xi yi zi            18 19 20    [m]
%    eta].';             21          []

Uiw = u(1); Viw = u(2); Wiw = u(3); % wind velocity wrt i frame in i coordinate
delt    = u(4); dela    = u(5);
dele    = u(6); delr    = u(7);
eta     = u(21);                                                           % span extension

[c,b,S,~,~,Tmax]=f18spec(0);
thrust=Tmax*delt;
Ft=[thrust 0 0].';

state13=u(8:20); % state vector
U =state13(1); V =state13(2); W =state13(3); vbGi=[U V W].';
P =state13(4); Q =state13(5); R =state13(6);
q0=state13(7); q1=state13(8); q2=state13(9); q3=state13(10); quat=[q0;q1;q2;q3];
[~,~,~,rho]=atmosisa(-state13(13));

vbWi=quat2dcm(quat.')*[Uiw Viw Wiw].'; % wind velocity wrt i frame in b coordinate
vbGw=vbGi-vbWi; Up=vbGw(1); Vp=vbGw(2); Wp=vbGw(3); % relative velocity in b coordinate

VT=norm(vbGw); % airspeed
alp=atan2(Wp,Up); bet=asin(Vp/VT);

[CX, CY, CZ, Cl, Cm, Cn] = f18aerocoeff(VT,alp,bet,P,Q,R,dela,dele,delr,eta);

q=0.5*rho*VT^2;
Xa=q*CX*S;   Ya=q*CY*S;   Za=q*CZ*S;   % aerodynamic formce [N]
La=q*Cl*S*b; Ma=q*Cm*S*c; Na=q*Cn*S*b; % aerodynamic moment [Nm]

FTa=[Xa;Ya;Za]; % aerodynamic formce [N]
MTa=[La;Ma;Na]; % aerodynamic moment [Nm]

F=FTa+Ft;
M=MTa;

sys=[F;M];