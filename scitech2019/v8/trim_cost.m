function cost = trim_cost(z,VT,h,eta)

alp = z(1);

U = VT*cos(alp);
V = 0;
W = VT*sin(alp);
v = [U;V;W];

P = 0;
Q = 0;
R = 0;
omega = [P;Q;R];

ph = 0;
th = alp;
ps = 0;
q = angle2quat(ps,th,ph).';

p_N = 0;
p_E = 0;
p_D = -h;
p = [p_N;p_E;p_D];

delt = z(2);
dela = 0;
dele = z(3);
delr = 0;

x = [v;omega;q;p];
u = [delt;dela;dele;delr];

dx = state_eq(x,u,eta);
weight = diag([1 1 1000]);

cost = sum(dx([1,3,5]).'*weight*dx([1,3,5]));