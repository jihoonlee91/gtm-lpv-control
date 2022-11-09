function tests = hinfidTest
%HINFIDTEST Test behavior of HINFID
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
rng(42); % make results reproducible
% create a second order mass spring damper system
k = 10;
c = 5;
m = 1;

A = [0 1; -k/m -c/m];
B = [0; 1/m];
C = [1 0];
D = 0;

G = ss(A,B,C,D);
testCase.TestData.sysCheck = G;
end

function testHinfid(testCase)
% create scheduling variable and lpvlfr system to identify
p = [];
LFRsys = lpvlfr([],drss(2,1,1));

% identify the system using hinfid
[~, gamma1] = hinfid({testCase.TestData.sysCheck},{p},LFRsys);

% check result!
% TODO: check why bode fails
% bode(LPRIDsys,p);
% hold on
% bode(testCase.TestData.sysCheck);

%% test using the same example using the paper:
% Linear fractional LPV model identification from local
% experiments: an H?-based optimization technique

% lfr model representation eq(3) using system description A
% (step 1, use the real values of the scheduling variables to estimate
m1 = 0.2;
m2 = 0.2;
K = 4;
f = 0.1;
f1 = 0.04;
f2 = 0.04;
k = 10;

A0 = [(-f-f1)/m1, f/m2, -k; f/m1, (-f-f2)/m2 k; 1/m1, -1/m2, 0];
Bw = [-f/m2; f/m2; -f/m2];
Bu = [K; 0; 0];
Cz = [0, 1/m2, 0];
Dzw = -1/m2;
Dzu = 0;
Co = [1/m1, 0, 0];
Dyw = 0;
Dyu = 0;

G = ss(A0, [Bw Bu], [Cz; Co], [Dzw Dzu; Dyw Dyu]);
Delta = preal('Delta', 'ct');

% create real system using the scheduling variables
p = 0:0.05:0.8; p = p';
sysreal = lpvlfr(Delta,G);


% lets identify the system
LTIs = cell(length(p),1);
pcell = cell(length(p),1);
for i=1:length(p)
    LTIs{i} = extractLocal(sysreal,p(i));
    pcell{i} = p(i);
end

rng(42);
c = 0.2;
Gdisturbed = ss(A0+c*randn(size(A0)), [Bw Bu]+c*randn(size([Bw Bu])), [Cz; Co]+c*randn(size([Cz; Co])), [Dzw Dzu; Dyw Dyu]+c*randn(size([Dzw Dzu; Dyw Dyu])));
sysrandom = lpvlfr(Delta,Gdisturbed);
[LFRIDsys, gamma2] = hinfid(LTIs,pcell,sysrandom);

bode(LFRIDsys,'r',p);
bode(sysreal,'b',p);
bode(sysrandom, 'y',p);
close all;

verifyTrue(testCase, gamma1 <10E-6 && gamma2 < 10E-6);
end


