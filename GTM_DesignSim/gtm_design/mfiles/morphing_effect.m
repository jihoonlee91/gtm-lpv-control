function [dinertia,dCG]  = morphing_effect(eta)

Cbar    = 0.9153;   % ft,    mean aerodynamic chord
le_mac  = 4.5462;   % ft,    leading edge of MAC
cg_pos0 = [-(le_mac + Cbar*0.2199) -0.1416/12 -0.9761]; % ft, initial CG (21.99% MAC), gear up

lb2slug = 1/32.174;

rcmbar = [cg_pos0(1); cg_pos0(2); cg_pos0(3)]; % [ft]
b = 6.8488; % span [ft]
Lambda = 28.43*pi/180; % sweep [rad]
Gamma = 5*pi/180; % diheral [rad]
m = 57.75*lb2slug; % [slg]
ddm = -0.81*lb2slug; % [slg]
ddrcm = [0.148; 0.628; 0.032]/12; % 3 by 1 [ft]
ddJxx = -0.25821; % slg-ft^2,
ddJyy = -0.01727; % slg-ft^2,
ddJzz = -0.27400; % slg-ft^2,
ddJxz = -0.00295; % slg-ft^2,
ddJyz = -0.01346; % slg-ft^2,
ddJxy = -0.05998; % slg-ft^2,
Jxxbar = 1.221;  % slg-ft^2,
Jyybar = 4.655;  % slg-ft^2,
Jzzbar = 5.587;  % slg-ft^2,
Jxzbar = 0.274;  % slg-ft^2,
Jyzbar = 0.0;    % slg-ft^2,
Jxybar = 0.006;  % slg-ft^2,
ddJ=[ ddJxx, -ddJxy, -ddJxz;
     -ddJxy,  ddJyy, -ddJyz;
     -ddJxz, -ddJyz,  ddJzz ];
Jbar=[Jxxbar, -Jxybar, -Jxzbar;
     -Jxybar,  Jyybar, -Jyzbar;
     -Jxzbar, -Jyzbar,  Jzzbar ];

etal = eta(1); % left morphing [%]
etar = eta(2); % right morphing [%]

ml = -ddm;
mr = ml;
mb = m-ml-mr;

drl = -(etal/25)*[b*tan(Gamma)/8;  b/8; b*tan(Gamma)/(8*cos(Lambda))]; % 3 by 1 
drr = -(etar/25)*[b*tan(Gamma)/8; -b/8; b*tan(Gamma)/(8*cos(Lambda))]; % 3 by 1
drcm = (ml/m)*drl+(mr/m)*drr;
dCG = drcm;
rcm = rcmbar+drcm;

rlbar = rcmbar-(m/ml-1)*ddrcm;
rrbar = diag([1,-1,1])*rlbar;
rbbar = (m*rcmbar-ml*rlbar-mr*rrbar)/mb;

rl = rlbar+drl;
rb = rbbar;
rr = rrbar+drr;

temp1 = rlbar-rcmbar;
temp2 = rbbar-rcmbar;
temp3 = rrbar-rcmbar;
temp4 = rbbar-(rcmbar+ddrcm);
temp5 = rrbar-(rcmbar+ddrcm);

Jlbar = -ddJ...
    -ml*(trace(temp1*temp1.')*eye(3)-temp1*temp1.')...
    -mb*(trace(temp2*temp2.')*eye(3)-temp2*temp2.')...
    -mr*(trace(temp3*temp3.')*eye(3)-temp3*temp3.')...
    +mb*(trace(temp4*temp4.')*eye(3)-temp4*temp4.')...
    +mr*(trace(temp5*temp5.')*eye(3)-temp5*temp5.');
Jlbar=-Jlbar;
Jrbar = [ Jlbar(1,1) -Jlbar(1,2)  Jlbar(1,3);
         -Jlbar(2,1)  Jlbar(2,2) -Jlbar(2,3);
          Jlbar(3,1) -Jlbar(3,2)  Jlbar(3,3)];
temp6 = rlbar-rcmbar;
temp7 = rrbar-rcmbar;
Jbbar = Jbar-Jlbar-ml*(trace((temp6)*(temp6).')*eye(3)-(temp6)*(temp6).')...
            -Jrbar-mr*(trace((temp7)*(temp7).')*eye(3)-(temp7)*(temp7).');
temp8 = rl-rcm;
temp9 = rb-rcm;
temp0 = rr-rcm;
Jl = Jlbar+ml*(trace((temp8)*(temp8).')*eye(3)-(temp8)*(temp8).');
Jb = Jbbar+mb*(trace((temp9)*(temp9).')*eye(3)-(temp9)*(temp9).');
Jr = Jrbar+mr*(trace((temp0)*(temp0).')*eye(3)-(temp0)*(temp0).');
J = Jl+Jb+Jr;
dJ = J-Jbar;
dinertia = [dJ(1,1) dJ(2,2) dJ(3,3) -dJ(1,3) -dJ(2,3) -dJ(1,2)];