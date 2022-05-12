% Define the time-varying real parameter.
% rho = pgrid('rho',linspace(-pi,pi,7));
% rho.RateBounds = [-5 5];
rhoData = linspace(-pi,pi,7);
domain = rgrid('rho',rhoData);
% Define the A, B, C, and D matrices of the LPV system in Equation (1)
% pcos = cos(rho);
% psin = sin(rho);
a = ureal('a',1,'Percentage',[-10 10]);
% A = [0.75 2 pcos psin;0 0.5 -psin pcos;0 0 -10 0; 0 0 0 -10];
B = [0 0 0;a 0 0;0 10 0;0 0 10];
C = [1 0 0 0;0 1 0 0];
D = zeros(2,3);
for i = 1:length(rhoData)
    A = [0.75 2 cos(rhoData(i)) sin(rhoData(i));0 0.5 -sin(rhoData(i)) cos(rhoData(i));0 0 -10 0; 0 0 0 -10];
    Data(:,:,i) = uss(A,B,C,D);
end
% Form the grid-based parameter-varying state-space system:
G = upss(Data,domain);
% Weights
%% Synthesis for gridded LPV systems
Wp = eye(2);
Wn = ss(10*tf([1 10],[1 1000]))*eye(2);
Wf = 1;
Wu = (1/280)*eye(2);
Wr = ss(tf(20,[1 0.2]))*eye(2);
% Control Interconnection Structure
systemnames = 'G Wp Wn Wf Wu Wr';
input_to_G = '[ Wf; u ]';
input_to_Wp = '[ G-Wr ]';
input_to_Wn = '[ dn ]';
input_to_Wf = '[ df ]';
input_to_Wu = '[ u ]';
input_to_Wr = '[ dr ]';
inputvar = '[ df; dr(2); dn(2); u(2)]';
outputvar = '[ Wu; Wp; G-Wr+Wn ]';
H = sysic
% Basis function,
b1 = basis(1,0);
bcos = basis(pcos,'rho',-psin);
bsin = basis(psin,'rho',pcos);
Xb = [b1;bcos;bsin];
Yb = Xb;
% LPV Rate-Bounded Control Design
opt = lpvsynOptions('BackOffFactor',1.02);
[Klpv,normlpv] = lpvsyn(H,2,2,Xb,Yb,opt);
Klpv
normlpv