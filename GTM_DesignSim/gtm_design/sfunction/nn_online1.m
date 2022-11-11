function [sys, x0, str, ts] = nn_online1(t,x,u,flag)

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
sizes.NumContStates  = 2*840*3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2*840*3;
sizes.NumInputs      = 3+3+1+6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(2*840*3,1);
str = [];
ts  = [0 0];

function sys = mdlDerivatives(~,~,u)
%      variable     index
% u = [V_T alp h   1 2 3
%      u_1 u_2 u_3 4 5 6
%      eta]        7

V_T =  u(1);
alp =  u(2);
h   = -u(3);
eta  = u(7);
e1 = u(8:9);
e2 = u(10:11);
e3 = u(12:13);
[~,a,~,~]=atmosisa(h);
mach = V_T/a;
mach_m = linspace(0.4,1.2,20);

gam1 = 41.4;
gam2 = 58.6;
gam3 = 39.7;

tsi = 1.5; % [s]
kd = 9.2/tsi; % [s^-1]
kp = (kd^2)/2;% [s^-2]

% A = [0,1;-kp,-kd];
b = [0;1];
P0 = [kd/(2*kp)+kp/(2*kd)*(1+1/kp), 1/(2*kp); 1/(2*kp), 1/(2*kd)*(1+1/kp)];

s1 = e1.'*P0*b;
s2 = e2.'*P0*b;
s3 = e3.'*P0*b;

% lam = 5;

% s1 = (kp+1)/(2*kp*kd)*(lam*e1(1)+e1(2));
% s2 = (kp+1)/(2*kp*kd)*(lam*e2(1)+e2(2));
% s3 = (kp+1)/(2*kp*kd)*(lam*e3(1)+e3(2));

beta = zeros(2*840,3);
for ii = 1:3
    u_i = u(3+ii);
    for kk = 0:20
    for ll = 0:4
    for mm = 0:1
    for nn = 0:3
    for rr = 0:1
        jj = kk*5*2*4*2+ll*2*4*2+mm*4*2+nn*2+rr+1;
        if kk == 0
            beta(jj,ii) = (alp^ll)*(eta^rr);
        else
            beta(jj,ii) = exp(-((mach-mach_m(kk))^2)/(2*0.2^2))*(alp^ll)*(eta^rr);
        end
    end
    end
    end
    end
    end
end

dw1 = -gam1*s1*beta(:,1);
dw2 = -gam2*s2*beta(:,2);
dw3 = -gam3*s3*beta(:,3);

% sys = [dw1/25;
%        dw2/25;
%        dw3/25];

% sys = [dw1/20;
%        dw2/50;
%        dw3/25];

% sys = [dw1/10;
%        dw2/40;
%        dw3/50];

sys = [dw1/2;
       dw2/50;
       dw3/30];

function sys = mdlOutputs(~,x,~)
% min(x)
% max(x)
sys = x;