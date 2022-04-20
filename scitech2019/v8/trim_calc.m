function [x_trim,u_trim,alp_trim,Err] = trim_calc(z_guess,VT0,h0,eta0)

global SPEC

fun = @(z)trim_cost(z,VT0,h0,eta0); % declare cost function for trim
A = [];
b = [];
Aeq = [];
beq = [];
lb = [deg2rad(-10);0;SPEC.dele_ll];
ub = [deg2rad(+10);1;SPEC.dele_ul];
c = [];
ceq = [];
nonlcon = [c,ceq];
options = optimoptions('fmincon','Display','off','Algorithm','sqp');
[Z,Err] = fmincon(fun,z_guess,A,b,Aeq,beq,lb,ub,nonlcon,options);

alp_trim = Z(1);

U_trim = VT0*cos(alp_trim);
V_trim = 0;
W_trim = VT0*sin(alp_trim);
v_trim = [U_trim;V_trim;W_trim];

P_trim = 0;
Q_trim = 0;
R_trim = 0;
omega_trim = [P_trim;Q_trim;R_trim];

ph_trim = 0;
th_trim = alp_trim;
ps_trim = 0;
q_trim = angle2quat(ps_trim,th_trim,ph_trim).';

p_N_trim = 0;
p_E_trim = 0;
p_D_trim = -h0;
p_trim = [p_N_trim;p_E_trim;p_D_trim];

delt_trim = Z(2);
dela_trim = 0;
dele_trim = Z(3);
delr_trim = 0;

x_trim = [v_trim;omega_trim;q_trim;p_trim];
u_trim = [delt_trim;dela_trim;dele_trim;delr_trim];