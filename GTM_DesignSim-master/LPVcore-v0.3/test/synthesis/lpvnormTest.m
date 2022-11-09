function tests = lpvnormTest
%LPVNORMTEST Verify lpvnorm algorithms for LTI systems
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%%
rng(1)

% Two uncoupled mass-spring-damper system
T1 = randn(2);
A1 = [0 1;-rand,-rand];
B1 = [0;1];
C1 = [0 1];
D1 = 0;

sys1 = ss2ss(ss(A1,B1,C1,D1),T1);

T2 = randn(2);
A2 = [0 1;-rand,-rand];
B2 = [0;1];
C2 = [0 1];
D2 = 0;

sys2 = ss2ss(ss(A2,B2,C2,D2),T2);

sys = blkdiag(sys1,sys2);
lpvsys = lpvss(sys);

lpvsys1 = lpvss(sys1);

testCase.TestData.sys1 = sys1;
testCase.TestData.sys = sys;
testCase.TestData.lpvsys = lpvsys;
testCase.TestData.lpvsys1 = lpvsys1;
end

function testL2(testCase)
    % Induced L2 norm and Hinf norm are equivalent for stable LTI systems
    sys = testCase.TestData.sys;
    lpvsys = testCase.TestData.lpvsys;

    l2LPV = lpvnorm(lpvsys,'l2','verbose',0);
    l2LTI = hinfnorm(sys);

    check = abs(l2LPV-l2LTI)<=10*sqrt(eps);

    verifyTrue(testCase, check); 
end

function testPassive(testCase)
    sys = testCase.TestData.sys;
    lpvsys = testCase.TestData.lpvsys;

    passiveLTI = isPassive(sys);
    passiveLPV = lpvnorm(lpvsys,'passive','verbose',0);

    check = passiveLTI && passiveLPV;

    verifyTrue(testCase, check);  
end
 
function testH2(testCase)
    % Generalized H2 norm is equivalent to H2 norm for siso systems, hence
    % why use sys1 here
    sys = testCase.TestData.sys1;
    lpvsys = testCase.TestData.lpvsys1;

    h2LPV = lpvnorm(lpvsys,'h2','verbose',0);
    h2LTI = norm(sys,2);

    check = abs(h2LPV-h2LTI)<=sqrt(eps);

    verifyTrue(testCase, check); 
end