function tests = pmatrixRolmipTest
%PMATRIXROLMIPTEST Test pmatrixToRolmip and rolmipToPmatrix functions
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%%
rng(1)

% Scheduling-variables
nom = [3 1];
delta = [2 5];
p = preal('p','Range',[nom(1)-delta(1),nom(1)+delta(1)]);
q = preal('q','Range',[nom(2)-delta(2),nom(2)+delta(2)]);

% Matrix
n = 5;
m = 3;

M0 = randn(n,m);
M1 = randn(n,m);
M2 = randn(n,m);

% pmatrix
M = M0+M1*p+M2*q;

% pmatrix --> rolmip
[Mr,st] = pmatrixToRolmip(M,'M');

% rolmip --> pmatrix
Mc = rolmipToPmatrix(Mr,M,st);

% Add to testCase
testCase.TestData.M = M;
testCase.TestData.Mr = Mr;
testCase.TestData.Mc = Mc;
testCase.TestData.nom = nom;
testCase.TestData.delta = delta;
end

function testPmatrixToRolmip(testCase)
    M  = testCase.TestData.M;
    Mr = testCase.TestData.Mr;
    nom = testCase.TestData.nom;
    delta = testCase.TestData.delta;

    pq = (2*rand(1,2)-1).*delta+nom;    % random scheduling-value (in range)

    verifyLessThanOrEqual(testCase, norm(feval(M,pq)-evalpar(Mr,pq)), 10*eps); 
end

function testRolmipToPmatrix(testCase)
    M  = testCase.TestData.M;
    Mc = testCase.TestData.Mc;

    check = M == Mc;

    verifyTrue(testCase, check); 
end