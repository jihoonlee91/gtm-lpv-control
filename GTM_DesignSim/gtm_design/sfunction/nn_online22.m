function [sys, x0, str, ts] = nn_online22(t,x,u,flag)

switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 3
        sys=mdlOutputs(t,x,u);
    case {2,4,9}
        sys=[];
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 21*3+7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

function sys = mdlOutputs(~,~,u)

V_T =  u(21*3+1);
alp =  u(21*3+2);
h   = -u(21*3+3);
u_ph = u(21*3+4);
u_th = u(21*3+5);
u_ps = u(21*3+6);
eta  = u(21*3+7);
[~,a,~,~]=atmosisa(h);
mach = V_T/a;
mach_m = linspace(0.4,0.8,21);

w = reshape(u(1:21*3),21,3);

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

u_ad1 = w(:,1).'*beta(:,1);
u_ad2 = w(:,2).'*beta(:,2);
u_ad3 = w(:,3).'*beta(:,3);

sys = [u_ad1;u_ad2;u_ad3];
% sys = zeros(3,1);