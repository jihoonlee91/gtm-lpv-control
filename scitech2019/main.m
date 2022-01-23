clear

%% plant
spec_data % load specification
aero_data % load aerodynamic data

V_grid = 20;
h_grid = 300;
eta1_grid = 0:0.5:1;
eta2_grid = 0:0.5:1;

V = pgrid('V',V_grid,[-5 5]);
eta1 = pgrid('eta1',eta1_grid,[-0.5 0.5]);
eta2 = pgrid('eta2',eta2_grid,[-0.5 0.5]);

tic
for ii = 1:numel(V_grid)
    for jj = 1:numel(h_grid)
        for nn = 1:numel(eta1_grid)
            for mm = 1:numel(eta2_grid)
                [Gss4,alp_trim(nn,mm),delt_trim(nn,mm),dele_trim(nn,mm)] = lti_model(V_grid(ii),h_grid(jj),eta1_grid(nn),eta2_grid(mm));
                Zalpha = Gss4.A(2,2)*V_grid(ii);
                Zdelta = Gss4.B(2,2)*V_grid(ii);
                Malpha = Gss4.A(3,2);
                Mdelta = Gss4.B(3,2);
                A = [Zalpha/V_grid(ii),1;
                    Malpha,0];
                B = [Zdelta/V_grid(ii);
                    Mdelta];
                C = [1,0;
                    0,1;
                    Zalpha,0];
                D = [0;
                    0;
                    Zdelta];
                Gss(:,:,nn,mm) = ss(A,B,C,D); % short-period mode
            end
        end
    end
end
toc

Domain = rgrid(eta1,eta2);
sys = pss(Gss,Domain);

%% synthesis
% Define weights:
We = tf(10,[1,0.1]);
Wu = 1/20;
Wa = 1/20;
% tf([1,0.4],[1,4000]);
% Wn = blkdiag(Wn1,Wn2,Wn3);
Wn = diag([0.1,0.1,0.2]);

wa = 105;
za = 0.6;
act = ss([0,1;-wa^2,-2*za*wa],[0;wa^2],[1,0],0);

% wr = 2*pi;
% zr = 0.7;

% wr = 2*pi*(2*eta1+1);
% zr = eta2

[freq,zeta] = damp(sys);
wr = freq(1);
zr = zeta(1);

Wr = ss([0,1;-wr^2,-2*zr*wr],[0;wr^2],[1,0],0);

% Form synthesis interconnection:
systemnames = 'sys act We Wu Wa Wn Wr';
inputvar = '[ref{1};dn{3};u]';
outputvar = '[Wu;Wa;We;ref;sys+Wn]';
input_to_sys = '[act]';
input_to_act = '[u]';
input_to_Wa = '[act]';
input_to_Wu = '[u]';
input_to_Wn = '[dn]';
input_to_We = '[sys(1)-Wr]';
input_to_Wr = '[ref]';
G = sysic;

% Synthesize two-degree of freedom controller.
b0 = basis(1,0);
b1 = basis(eta1,'eta1',1);
b2 = basis(eta2,'eta2',1);
Xb = [b0;b1;b1^2;b2;b2^2;b1*b2];
Yb = Xb;
opt = lpvsynOptions('BackOffFactor',1.02);
nmeas = 4; % # of measurements
ncont = 1; % # of controls
tic
[Knr,Gamma,Info] = lpvsyn(G,nmeas,ncont,Xb,Yb,opt);
toc
Gamma

% Insert Knr into the weighted interconnection:
CLICnr = lft(G,Knr);

% Form closed-loop sytem
systemnames = 'sys act Knr';
inputvar = '[r{1}]';
outputvar = '[sys]';
input_to_sys = '[act]';
input_to_act = '[Knr]';
input_to_Knr = '[r;sys]';
CL = sysic;

%% step response
opt = stepDataOptions('StepAmplitude',pi/180);
fig1 = figure('position',[-1918 -158 958 993]);
step(CL,opt,4)
drawnow

%% bodeplot
fig2 = figure('position',[-958 -158 958 993]);
bodemag(sys,{0.1,100},'b')
hold on
bodemag(CL,{0.1,100},'r--')
legend('Open-loop','Closed-loop','location','best')
grid minor
hold off
drawnow

%% lpv simulation
dt = 0.01;
t = (0:dt:2)';
u = ones(size(t))*pi/180;
% Define the trajectories of the parameters:
ptraj.time = t;
ptraj.eta1 = t/2;
ptraj.eta2 = 1-t/2;
ptraj.eta1Dot = 1/2+t*0;
ptraj.eta2Dot = -1/2+t*0;
fig3 = figure('position',[2562 -147 958 1113]);
plot(t,ptraj.eta1,t,ptraj.eta2)
grid on
legend('\eta_1','\eta_2')
drawnow

% Perform LPV simulation:
fig4 = figure('position',[3522 -147 958 1113]);
tic
lpvlsim(CL,ptraj,u,t);
toc
drawnow