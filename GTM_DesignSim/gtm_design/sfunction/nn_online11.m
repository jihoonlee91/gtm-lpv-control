function [sys, x0, str, ts] = nn_online11(t,x,u,flag)

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
sizes.NumContStates  = 21*3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 21*3;
sizes.NumInputs      = 3+3+1+6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = zeros(21*3,1);
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
mach_m = linspace(0.4,0.8,21);

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

beta = zeros(21,3);
for ii = 1:3
%     u_i = u(3+ii);
    for kk = 0:20
%     for ll = 0:4
%     for mm = 0:1
%     for nn = 0:3
%     for rr = 0:1
        jj = kk+1;
        if kk == 0
            beta(jj,ii) = 1;
        else
            beta(jj,ii) = exp(-((mach-mach_m(kk))^2)/(2*1^2));
        end
%     end
%     end
%     end
%     end
    end
end

% beta(1:10,1)

% norm(e1) > deg2rad(3)
% norm(e2) > deg2rad(3)
% norm(e3) > deg2rad(3)

% if norm(e1) > deg2rad(3)
    dw1 = -gam1*s1*beta(:,1);
% else
%     dw1 = zeros(2*840,1);
% end
% 
% if norm(e2) > deg2rad(3)
    dw2 = -gam2*s2*beta(:,2);
% else
%     dw2 = zeros(2*840,1);
% end
% 
% if norm(e3) > deg2rad(3)
    dw3 = -gam3*s3*beta(:,3);
% else
%     dw3 = zeros(2*840,1);
% end
% 

sys = [dw1;dw2;dw3];
% norm(sys)

function sys = mdlOutputs(~,x,~)
% min(x)
% max(x)
sys = x;