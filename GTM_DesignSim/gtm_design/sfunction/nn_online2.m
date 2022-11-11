function [sys, x0, str, ts] = nn_online2(t,x,u,flag)

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
sizes.NumInputs      = 2*840*3+7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

function sys = mdlOutputs(~,~,u)

V_T =  u(2*840*3+1);
alp =  u(2*840*3+2);
h   = -u(2*840*3+3);
u_ph = u(2*840*3+4);
u_th = u(2*840*3+5);
u_ps = u(2*840*3+6);
eta  = u(2*840*3+7);
[~,a,~,~]=atmosisa(h);
mach = V_T/a;
mach_m = linspace(0.4,1.2,20);

w = reshape(u(1:2*840*3),2*840,3);

beta = zeros(2*840,3);
for ii = 1:3
    u_i = u(2*840*3+3+ii);
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

u_ad1 = w(:,1).'*beta(:,1);
u_ad2 = w(:,2).'*beta(:,2);
u_ad3 = w(:,3).'*beta(:,3);

sys = [u_ad1;u_ad2;u_ad3];
% sys = zeros(3,1);