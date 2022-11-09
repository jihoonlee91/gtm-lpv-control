function tests = pmatrixTest
%PMATRIXTEST Test behavior of PMATRIX
tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
tm1 = timemap(randi(201, 1, 10) - 101, ...
    'dt', ...
    'Name', {'a', 'b'}, ...
    'Unit', {'-', 'cm'}, ...
    'Range', {[-100, 100], [0, 1]}, ...
    'RateBound', {[-1, 1], [0, 1]});
tm2 = timemap(randi(201, 1, 10) - 101, ...
    'dt', ...
    'Name', {'a', 'c'}, ...
    'Unit', {'-', 'm'}, ...
    'Range', {[-100, 100], [-Inf, +Inf]}, ...
    'RateBound', {[-Inf, +Inf], [-2, 2]});

tm3 = timemap(randi(201, 1, 10), ...
    'ct', ...
    'Name', {'a', 'c'}, ...
    'Unit', {'-', 'm'}, ...
    'Range', {[-100, 100], [-Inf, +Inf]});
tm4 = timemap(randi(201, 1, 10), ...
    'ct', ...
    'Name', {'a', 'c'}, ...
    'Unit', {'-', 'm'}, ...
    'Range', {[-100, 100], [-Inf, +Inf]});

testCase.TestData.P1 = pmatrix(randn(4, 4, 2), ...
    'BasisType', 'affine', ...
    'SchedulingTimeMap', tm1);
testCase.TestData.P2 = pmatrix(randn(4, 4, 2), ...
    'BasisType', 'affine', ...
    'SchedulingTimeMap', tm2);

testCase.TestData.P3 = pmatrix(randn(4, 4, 2), ...
    'BasisType', 'custom', ...
    'BasisParametrization', ...
    {@(rho) sin(rho(:, 1)), @(rho) cos(rho(:, 1))}, ...
    'SchedulingTimeMap', tm3);
testCase.TestData.P4 = pmatrix(randn(4, 4, 2), ...
    'BasisType', 'poly', ...
    'BasisParametrization', {[0, 1, 2, 4], [0, 0, 0, 4, 2, 2, 1]}, ...
    'SchedulingTimeMap', tm4);
testCase.TestData.P5 = pmatrix(randn(4,4,2), ...
    'BasisType', 'interp', ...
    'BasisParametrization', {{{[0 1], [ 1 2 3]},[ 0 0 0; 0 1 0]}, {{[0 1], [ 1 2 3]},[ 0 0 0; 0 0 1]} }, ...
    'SchedulingTimeMap', tm4);

% A collection of function handles to test overloaded operators
testCase.TestData.ops = {@(A, B) A + B, @(A, B) A * B, ...
    @(A, B) [A, B], @(A, B) A .* B, @(A, B) A - B, @(A, B) [A; B], ...
    @(A, B) blkdiag(A, B), @(A, B) A - B};
end

%% Test functions
function testCreation(testCase)
    % Test direction creation
    
    % Duplicate bfuncs should be collapsed
    P = pmatrix(cat(3, 1, 1, 1, 1), ...
        {pbconst, pbconst, pbconst, pbconst}, ...
        'SchedulingTimeMap', timemap);
    verifyTrue(testCase, freeze(P) == 4 && ...
        numel(P.bfuncs) == 1);
    
    P = pmatrix(cat(3, 1, 2, 3), ...
        {pbaffine(1), pbaffine(1), pbaffine(1)}, ...
        'SchedulingTimeMap', timemap(0, 'ct', 'Dimension', 2));
    verifyTrue(testCase, P == 6 * preal('p1', 'ct'));
    
    % Cannot pass basis function explicitly AND use
    % Basis{Type/Parametrization}
    verifyError(testCase, @() pmatrix(1, {pbconst}, ...
        'BasisType', 'affine'), 'LPVcore:pmat:Constructor:InputMismatch');
    verifyError(testCase, @() pmatrix(1, {pbaffine(1)}, ...
        'BasisParametrization', {1}), 'LPVcore:pmat:Constructor:InputMismatch');
    
    % Invalid basis parametrizations
    verifyError(testCase, @() pmatrix(1, ...
        'BasisType', 'custom', ...
        'BasisParametrization', {}), 'LPVcore:pmat:Constructor:InputMismatch');
    verifyError(testCase, @() pmatrix(1, ...
        'BasisType', 'custom', ...
        'BasisParametrization', {1, 2}), 'LPVcore:pmat:Constructor:InputMismatch');
    verifyError(testCase, @() pmatrix(1, ...
        'BasisType', 'custom', ...
        'BasisParametrization', {'not-a-valid-pbcustom'}), '');
end

function testAffine(testCase)
    % Test creation of affine
    p = preal('p');
    P = pmatrix(ones(1, 1, 2));
    verifyTrue(testCase, pmatrix(magic(2)) == magic(2) * p);
    verifyTrue(testCase, P == 1 + p);
    % Cannot use more than 2 matrices without more details on timemap
    verifyError(testCase, @() pmatrix(ones(1, 1, 3)),  'LPVcore:pmat:Constructor:InputMismatch');
    
    % Using 2 scheduling variables
    q = preal('q');
    tm = timemap(0, 'ct', 'Name', {'p', 'q'});
    P = pmatrix(ones(1, 1, 3), 'SchedulingTimeMap', tm);
    verifyTrue(testCase, P == p + q + 1);
    P = pmatrix(ones(1, 1, 2), 'SchedulingTimeMap', tm, ...
        'BasisParametrization', {1, 2});
    verifyTrue(testCase, P == p + q);
    
    % Using 1 scheduling variable with 2 shifts
    tm = timemap([0, 1], 'ct');
    dp = pdiff(p, 1);
    P = pmatrix(ones(1, 1, 3), 'SchedulingTimeMap', tm);
    verifyTrue(testCase, P == p + dp + 1);
end

function testCommonRho(testCase)
%also tests eq() minus() and plus()
P1 = testCase.TestData.P1;
P2 = testCase.TestData.P2;

[P1d, P2d] = pmatrix.commonrho_(P1, P2);
verifyTrue(testCase, P1 - 10 == P1d - 10);
verifyTrue(testCase, P2 - 10 == P2d - 10);
end

function testPlus(testCase)
P1 = testCase.TestData.P1;
P2 = testCase.TestData.P2;

rho1 = ones(1, P1.timemap.nrho);
rho2 = ones(1, P2.timemap.nrho);
P3 = P1 + P2;
rho3 = ones(1, P3.timemap.nrho);

verifyTrue(testCase, all(all(abs(P1.feval(rho1) + P2.feval(rho2) - P3.feval(rho3)) <= 1E-10)));

verifyTrue(testCase, (P1 + P2) == (P2 + P1));
verifyTrue(testCase, pmatrix(10) + pmatrix(5) == 15 * preal('p'));
verifyTrue(testCase, P1 + 0 == P1);
verifyTrue(testCase, P1 + P2 + 10 == 10 + P2 + P1);

% Repeat test with other pair of test data pmatrix objects
P1 = testCase.TestData.P3;
P2 = testCase.TestData.P4;

rho1 = ones(1, P1.timemap.nrho);
rho2 = ones(1, P2.timemap.nrho);
P3 = P1 + P2;
rho3 = ones(1, P3.timemap.nrho);

verifyTrue(testCase, all(all(abs(P1.feval(rho1) + P2.feval(rho2) - P3.feval(rho3)) <= 1E-10)));

verifyTrue(testCase, (P1 + P2) == (P2 + P1));
verifyTrue(testCase, pmatrix(10) + pmatrix(5) == 15 * preal('p'));
verifyTrue(testCase, P1 + 0 == P1);
verifyTrue(testCase, P1 + P2 + 10 == 10 + P2 + P1);

% Incompatible sizes
verifyError(testCase, @() P1 + [P1, P2], 'MATLAB:sizeDimensionsMustMatch');
verifyError(testCase, @() [] + P1, 'MATLAB:sizeDimensionsMustMatch');

% Multiple scheduling signals
a = preal('a');
b = preal('b');
c = preal('c');

m1 = a*c;
m2 = a^2*b;

pM1 = [1, 5];
pM2 = [1, 3];
pM = [1, 3, 5];

verifyEqual(testCase, freeze([m1, m2], pM), [freeze(m1, pM1), freeze(m2, pM2)], ...
    'AbsTol', 1E-10);
end

function testMinus(testCase)
P1 = testCase.TestData.P1;
P2 = testCase.TestData.P2;
verifyTrue(testCase, -(P1 - P2) == P2 - P1);
verifyTrue(testCase, P1 - 0 == P1);
end

function testKron(testCase)
P1 = testCase.TestData.P3;
P2 = testCase.TestData.P4;

[n, m] = size(P2);
P3 = kron(P1, P2);
verifyTrue(testCase, P3(1:n, 1:m) == P1(1, 1) * P2);

[n, m] = size(P1);
P3 = kron(P2, P1);
verifyTrue(testCase, P3(1:n, 1:m) == P2(1, 1) * P1);
end

function testMPower(testCase)
P1 = testCase.TestData.P1;

%verifyTrue(testCase, P1^0 == pmatrix(eye(size(P1, 1))));
%incorrect since returns a ct timemap instead of original timemap.
verifyTrue(testCase, P1^2 == P1 * P1);
verifyTrue(testCase, P1^5 == P1^2 * P1^3);

P3 = testCase.TestData.P3;

%verifyTrue(testCase, P3^0 == pmatrix(eye(size(P3, 1))));
%incorrect since returns a ct timemap instead of original timemap.
verifyTrue(testCase, P3^2 == P3 * P3);
% ...however, we can verify the magnitude of the difference is small
Psmall = P3^5 - P3^2 * P3^3;
verifyTrue(testCase, max(max(abs(Psmall.feval(1)))) <= 1E-10);
end

function testBlkDiag(testCase)
P1 = testCase.TestData.P1;
P2 = testCase.TestData.P2;

P3 = blkdiag(P1, P2);
verifyTrue(testCase, ...
    P3(1, 1) == P1(1, 1) && P3(end, end) == P2(end, end));
end

function testDiag(testCase)
    P1 = testCase.TestData.P1;

    x = diag(P1);
    verifyTrue(testCase, ...
        P1(1, 1) == x(1, 1) && P1(2, 2) == x(2, 1) && P1(end, end) == x(end, 1));
    
    % k = 0
    verifyTrue(testCase, ...
        diag(preal('p')) == preal('p'));
    verifyTrue(testCase, ...
        diag(diag(P1) == x));
    
    % k != 0
    p = preal('p');
    q = preal('q');
    A = [p, q; 1, 2];
    verifyTrue(testCase, ...
        diag(A, 1) == q);
    verifyTrue(testCase, ...
        diag(A, -1) == 1);
    v = [p, q, 1];
    verifyTrue(testCase, ...
        all(size(diag(v, 1)) == [4, 4]))
    
    % non-square pmatrix
    A = randn(2, 3) + randn(2, 3) * p + randn(2, 3) * q^2;
    f = randn(1, 2);
    args = {{}, {1}, {-1}};
    for i=1:numel(args)
        arg = args{i};
        verifyEqual(testCase, ...
            diag(freeze(A, f), arg{:}), freeze(diag(A, arg{:}), f), ...
            'AbsTol', 2 * eps);
    end
    
    % k is larger than size returns empty pmatrix
    verifyTrue(testCase, ...
        isempty(diag(A, 100)));
end

function testCat(testCase)
% Test concatenation of pmatrix objects
p1 = preal('p1', 'ct', 'Range', [-1, 9]);
p2 = preal('p2', 'ct', 'Range', [-2, 1]);

A1 = [1, 1 + p1; -3 * p2, -6 * p1];
A2 = [1, 1 + p2; -3 * p1, -6 * p2];

verifyTrue(testCase, ...
    all(A1.timemap == A2.timemap) && ...
    all(A1.timemap.Range{1} == [-1, 9]) && ...
    all(A1.timemap.Range{2} == [-2, 1]) && ...
    all(A2.timemap.Range{1} == [-1, 9]) && ...
    all(A2.timemap.Range{2} == [-2, 1]));
end

function testCat2(testCase)
    % Test re-indexing of pmatrix terms when concatening two pmatrices
    p = preal('p', 'dt');
    q = preal('q', 'dt');
    
    [~, IA, IB] = cat(3, 1 + p, 1 + p);
    verifyTrue(testCase, isequal([IA; IB], [1; 2; 1; 2]));
    
    [~, IA, IB] = cat(3, 1, 1 + p);
    verifyTrue(testCase, isequal([IA; IB], [1; 1; 2]));
    
    [~, IA, IB] = cat(3, 1 + p + p^2, 1 + p^2);
    verifyTrue(testCase, isequal([IA; IB], [1; 2; 3; 1; 3]));
    
    [~, IA, IB] = cat(3, 1 + p + p * q, 1 + q + p * q);
    verifyTrue(testCase, isequal([IA; IB], [1; 2; 3; 1; 4; 3]));
    
    P1 = pmatrix(randn(1, 1, 20), 'BasisType', 'poly');
    P2 = pmatrix(randn(1, 1, 20), 'BasisType', 'poly');
    [~, IA, IB] = cat(3, P1, P2);
    verifyTrue(testCase, isequal(IA, (1:20)') && isequal(IA, IB));
end

function testIsConst(testCase)
P1 = testCase.TestData.P1;



verifyTrue(testCase, ~isconst(P1));
verifyTrue(testCase, isconst(0 * preal('p')));
verifyTrue(testCase, isconst(pzeros(2, 2)));
verifyTrue(testCase, isconst(pmatrix([])));

P = pmatrix([1 2; 3 4],...
           'BasisType',  'custom',...               
           'BasisParametrization', {@(rho) 1},...
           'SchedulingTimeMap', P1.timemap);
%due to current implementation of isconst() this is not a constant pmatrix
verifyTrue(testCase, ~isconst(P));

       
end

function testIsAffine(testCase)
P1 = testCase.TestData.P1;

verifyTrue(testCase, isaffine(P1));
verifyTrue(testCase, ~isaffine(P1 * P1));
verifyTrue(testCase, isaffine(0 * preal('p')));
verifyTrue(testCase, isaffine(pmatrix([])));

P = pmatrix([1 2; 3 4],...
           'BasisType',  'custom',...               
           'BasisParametrization', {@(rho) rho(:,1)},...
           'SchedulingTimeMap', P1.timemap);
%due to current implementation of isconst() this is not an affine pmatrix
verifyTrue(testCase, ~isaffine(P));

end

function testPEye(testCase)
P = peye(10);
verifyTrue(testCase, P == eye(10));
verifyTrue(testCase, isconst(P) && isaffine(P) && P.Np == 0);
end

function testPReal(testCase)
p = preal('p', 'Range', [-1, 1], 'Unit', 'cm');
verifyTrue(testCase, all(p.timemap.Range{1} == [-1, 1]));
p = preal('p', 'ct', 'Range', [-1, 1], 'Unit', 'cm');
verifyTrue(testCase, all(p.timemap.Range{1} == [-1, 1]));
p = preal('p', 'dt', 'Range', [-1, 1], 'Unit', 'cm', 'Dynamic', -1);
verifyTrue(testCase, strcmp(p.timemap.Unit{1}, 'cm') && ...
    p.timemap.Map == -1);

p = preal('p', 'Range', [-1, 1], 'Unit', 'cm', 'Dynamic', 0);
q = preal('q', 'Range', [-2, 2], 'Unit', 'm', 'Dynamic', 1);

ops = testCase.TestData.ops;

for i=1:numel(ops)
    op = ops{i};
    c = op(p, q);
    verifyTrue(testCase, all(c.timemap.Range{1} == [-1, 1]) && ...
        all(c.timemap.Range{2} == [-2, 2]));
    verifyTrue(testCase, strcmp(c.timemap.Unit{1}, 'cm') && ...
        strcmp(c.timemap.Unit{2}, 'm'));
    verifyTrue(testCase, strcmp(c.timemap.Name{1}, 'p') && ...
        strcmp(c.timemap.Name{2}, 'q'));
end
end

function testFreeze(testCase)
p = preal('p', 'ct');
A0 = randn(2, 2);
A1 = randn(2, 2);

A = A0 + p * A1 + pdiff(p, 1) * randn(2, 2);

verifyTrue(testCase, all(A0 == freeze(A, 0), 'all') && ...
    all(A0 + A1 == freeze(A, 1), 'all') && ...
    all(A0 - A1 == freeze(A, -1), 'all'));
end

function testIsEmpty(testCase)
    % Operations on empty matrix return empty pmatrix
    A = pmatrix([]);
    ops = testCase.TestData.ops;
    for i=1:numel(ops)
        op = ops{i};
        C = op(A, A);
        verifyTrue(testCase, isempty(C));
    end
    
    % Some dimension non-zero
    verifyTrue(testCase, isempty(pmatrix(randn(10, 0))));
    verifyTrue(testCase, ~isempty(pmatrix(1)));
end

function testConstant(testCase)
    % TESTCONSTANT Test behaviour with constant pmatrix objects (i.e., Np = 0)
    A = prandn(2);
    p = preal('p', 'dt');
    q = preal('q', 'dt');
    B = randn(2) * p + randn(2) + q * randn(2);
    ops = testCase.TestData.ops;
    for i=1:numel(ops)
        op = ops{i}; C = op(A, B);
        verifyTrue(testCase, A.Np == 0);
        verifyTrue(testCase, C.Np == 2);
        verifyTrue(testCase, strcmp(C.timemap.Domain, B.timemap.Domain))
        Af = freeze(A);
        Bf = freeze(B, [1, 1]);
        Cf = freeze(C, [1, 1]);
        verifyTrue(testCase, max(max(abs(op(Af, Bf) - Cf))) < 1E-10);
    end
end

function testSum(testCase)
% Compare frozen pmatrix
P1 = testCase.TestData.P1;
P2 = testCase.TestData.P2;
P3 = testCase.TestData.P3;
P4 = testCase.TestData.P4;
P = {P1, P2, P3, P4};
for i=1:numel(P)
    p = ones(1, P{i}.Np);
    verifyTrue(testCase, norm(sum(freeze(P{i}, p)) - freeze(sum(P{i}), p)) < 1E-10);
    verifyTrue(testCase, norm(sum(freeze(P{i}, p), 1) - freeze(sum(P{i}, 1), p)) < 1E-10);
    verifyTrue(testCase, norm(sum(freeze(P{i}, p), 2) - freeze(sum(P{i}, 2), p)) < 1E-10);
    verifyTrue(testCase, norm(sum(freeze(P{i}, p), 'all') - freeze(sum(P{i}, 'all'), p)) < 1E-10);
end
end


function testDivide(testCase)
    %% Right matrix divide
    A = randn(10,2);

    Ap = pmatrix(A);
    B = [3 0;0 4];

    C = A/B;        % MATLAB
    Cp = Ap/B;      % LPVcore
    verifyTrue(testCase, norm(C - Cp.matrices) < 1E-10);

    %% Left matrix divide
    A = randn(2,10);

    Ap = pmatrix(A);
    B = [3 0;0 4];

    C = B\A;        % MATLAB
    Cp = B\Ap;      % LPVcore
    verifyTrue(testCase, norm(C - Cp.matrices) < 1E-10);

    %% Right array divide
    A = randn(2,2);

    Ap = pmatrix(A);
    B = [3 4];

    C = A./B;        % MATLAB
    Cp = Ap./B;      % LPVcore
    verifyTrue(testCase, norm(C - Cp.matrices) < 1E-10);

    %% Left array divide
    A = randn(2,2);

    Ap = pmatrix(A);
    B = [3 4];

    C = B.\A;        % MATLAB
    Cp = B.\Ap;      % LPVcore
    verifyTrue(testCase, norm(C - Cp.matrices) < 1E-10);
end

function testPeval(testCase)
    % TESTPEVAL Test evaluation of pmatrix for trajectory for p

    % Constant
    A = double2pmatrix(1);
    P = zeros(10, 0);
    E = peval(A, P);
    verifyTrue(testCase, all(size(E, 3) == size(P, 1)));
    
    % With shifted of the scheduling signal
    p = preal('p', 'dt');
    A = p + pshift(p, -1) + pshift(p, 1);
    P = (1:10)';
    E = peval(A, P);
    E_true = [6; 9; 12; 15; 18; 21; 24; 27];
    verifyEqual(testCase, E(:), E_true);
end

function testFeval(testCase)
    % TESTFEVAL Test evaluation of pmatrix for trajectory of rho (ext.
    % scheduling signal)
    
    rho = timemap([0, 1], 'ct');
    s = @(rho) sin(2 * pi * rho(:, 1));
    c = @(rho) cos(pi * rho(:, 2));
    A = pmatrix(cat(3, 1, -1), ...
        'BasisType', 'custom', ...
        'BasisParametrization', {s, c}, ...
        'SchedulingTimeMap', rho);
    % pmatrix can be compared with equivalent function handle
    A_fh = @(rho) s(rho) - c(rho);
    
    % Single rho
    rho = randn(1, A.timemap.nrho);
    verifyEqual(testCase, feval(A, rho), A_fh(rho));
    
    % Two rows of rho (i.e., corresponding to two time instances)
    % FEVAL returns a 3D-array with a 2D matrix for each row
    rho = randn(2, A.timemap.nrho);
    A_feval = feval(A, rho);
    verifyEqual(testCase, A_feval(:, :, 1), A_fh(rho(1, :)));
    verifyEqual(testCase, A_feval(:, :, 2), A_fh(rho(2, :)));
    verifyEqual(testCase, size(A_feval), ...
        [size(A, 1), size(A, 2), size(rho, 1)]);
    
    % FEVAL expands scalars
    rho = 1;
    verifyEqual(testCase, feval(A, rho), A_fh([rho, rho]));
    
    % Scalar expansion if multiple time instances are given
    rho = [1; 2];
    A_feval = feval(A, rho);
    verifyEqual(testCase, A_feval(:, :, 1), A_fh([rho(1), rho(1)]));
    verifyEqual(testCase, A_feval(:, :, 2), A_fh([rho(2), rho(2)]));
    verifyEqual(testCase, size(A_feval), ...
        [size(A, 1), size(A, 2), size(rho, 1)]);
end

function testReshape(testCase)
    A = pmatrix(rand(3,4));
    p = 2;
    verifyEqual(testCase, feval(reshape(A,[2,6]),p), reshape(feval(A,p),[2,6]))
    verifyEqual(testCase, reshape(A,[2,6]) , reshape(A, 2, 6))
    verifyError(testCase, @() reshape(A, [2, 2, 3]), 'LPVcore:DimConvention')
    verifyError(testCase, @() reshape(A, [2, 3]), 'MATLAB:getReshapeDims:notSameNumel')
end

function testIsPoly(testCase)
    % TESTISPOLY The ISPOLY function should return true if and only if all basis
    % functions are monomials (pbconst, pbaffine, pbpoly)
    
    p = preal('p', 'dt');
    q = preal('q', 'dt');

    % Positive cases
    verifyTrue(testCase, ispoly(p^0));
    verifyTrue(testCase, ispoly(p));
    verifyTrue(testCase, ispoly(pshift(p, 1)));
    verifyTrue(testCase, ispoly(p^2 - q^3));
    
    % Negative cases
    s = @(rho) sin(rho(:, 1));
    A = pmatrix(1, ...
        'BasisType', 'custom', ...
        'BasisParametrization', {pbcustom(s)}, ...
        'SchedulingDomain', 'dt');
    verifyFalse(testCase, ispoly(A));
    verifyFalse(testCase, ispoly(A + p));
end

function testEmpty(testCase)
    verifyEqual(testCase,pmatrix.empty(0,0),pmatrix([]))
    verifyEqual(testCase,pmatrix.empty(0,0,0),pmatrix([], {pbconst}))
    
    verifyError(testCase,@() pmatrix.empty(0,0,0,0), 'MATLAB:InputParser:ArgumentFailedValidation')
    verifyError(testCase,@() pmatrix.empty(1,1),     'MATLAB:class:emptyMustBeZero')
end

function testPDiff(testCase)
    p = preal('p', 'ct');
    q = preal('q', 'ct');
    pd = preal('p', 'ct', 'Dynamic', 1);
    qd = preal('q', 'ct', 'Dynamic', 1);
    
    % Chain rule for pbpoly
    verifyTrue(testCase, pdiff(p, 1) == pd);
    verifyEqual(testCase, feval(pdiff(p^2, 1), [1, 2]), feval(2 * p * pd, [1, 2]));
    verifyTrue(testCase, pdiff(p + q, 1) == pd + qd);
    verifyEqual(testCase, ...
        feval(pdiff((p+q)^2, 1), [1, 2, 3, 4]), ...
        feval(2 * (p * pd + p * qd + pd * q + q * qd), [1, 2, 3, 4]));
    verifyTrue(testCase, pdiff(p + 1, 1) == pd);
    verifyTrue(testCase, pdiff(p + 1, 2) == pdiff(pd, 1));
    verifyTrue(testCase, pdiff(p^2 + p + 1, 1) == 2 * pd * p + pd);
    
    % Custom basis functions are not supported yet and should throw error
    phi = @(rho) rho(:, 1);
    A = pmatrix(1, {pbcustom(phi)});
    verifyError(testCase, @() pdiff(A, 1), 'LPVcore:pmat:wrongBasisFunctions');
    
    % Negative order throws error
    verifyError(testCase, @() pdiff(p, -1), 'LPVcore:pmat:NonPositiveInteger');
    verifyError(testCase, @() pdiff(p, Inf), 'LPVcore:pmat:NonPositiveInteger');
    verifyError(testCase, @() pdiff(p, NaN), 'LPVcore:pmat:NonPositiveInteger');
end

function testSubsRef(testCase)
    % Test SUBSREF, e.g., A(1), A(:), A(1, 1), ...
    p = preal('p');
    
    A = randn(2) + p * randn(2);
    Ac = freeze(A, 1);
    
    % Indices (linear and row-column based) 
    args = {{1}, {2}, {3}, {4}, {1, 1}, {1, 2}, {2, 1}, {2, 2}};
    for i=1:numel(args)
        a = args{i};
        verifyEqual(testCase, freeze(A(a{:}), 1), Ac(a{:}));
    end
    
    % Vectorization
    verifyEqual(testCase, freeze(A(:), 1), Ac(:));
end

function testSubsAsgn(testCase)
    % Test SUBSASGN, e.g., A(1) = p, A(1, 1) = p, ...
    p = preal('p');
    
    % Indices (linear and row-column based) 
    args = {{1}, {2}, {3}, {4}, {1, 1}, {1, 2}, {2, 1}, {2, 2}, ...
        {[1, 2], 1}, {':', 2}, {[1 2], ':'}};
    for i=1:numel(args)
        A = randn(2) + p * randn(2);
        Ac = freeze(A, 1);
        
        a = args{i};
        
        % Assign a matrix
        % Sample a random matrix for assignment
        R = randn(2);
        R = R(a{:});
        % Assign R to pmatrix and matrix
        A(a{:}) = R;
        Ac(a{:}) = R;
        verifyEqual(testCase, freeze(A, 1), Ac);
        
        % Assign a scalar
        r = randn;
        A(a{:}) = r;
        Ac(a{:}) = r;
        verifyEqual(testCase, freeze(A, 1), Ac);
    end
    
    % With vectorization
    A = randn(2) + p * randn(2);
    Ac = freeze(A, 1);
    
    % Assigning a matrix
    R = randn(2);
    A(:) = R;
    Ac(:) = R;
    verifyEqual(testCase, freeze(A, 1), Ac);
    
    % Assigning a scalar
    r = randn;
    A(:) = r;
    Ac(:) = r;
    verifyEqual(testCase, freeze(A, 1), Ac);
    
    % Assigning outside of initial size
    A = p;  % 1-by-1
    A(2, 2) = p;
    A(5, 7) = p^2;
    A(10:12, [5, 7, 9]) = 1 - p^3; %#ok<NASGU>
end

function testSimplify(testCase)
    p = preal('p', 'ct');
    q = preal('q', 'ct');
    a = p+p+q;
    b = p+q+p;
    c = q+p+p;
    
    verifyNotEqual(testCase, b,a)
    verifyEqual(testCase, simplify(b), simplify(a))
    verifyEqual(testCase, b, c)    
    
    %% testing in construction
    sz = 3;
    mat = rand(2,3, 4);
    bfs = {pbpoly(zeros(sz)), pbaffine(0), pbconst(), pbcustom(@(rho) pi())};
    matsp = mat;
    matsp(:,:,1) = matsp(:,:,1)*sz;
    matsp(:,:,4) = matsp(:,:,4)*pi();
    pmat = pmatrix(mat, bfs, 'SchedulingName', {'p1', 'p2', 'p3'});
    
    verifyEqual(testCase, feval(pmat, pi*ones(1,sz)), sum(matsp,3), 'relTol', 1e-9)
    
end

function testNumel(testCase)
    verifyEqual(testCase, numel(pmatrix([])), 0)
    p = preal('p', 'ct');
    verifyEqual(testCase, numel(p), 1)
    verifyEqual(testCase, numel((p+1)),1)
    verifyEqual(testCase, numel(randn(2)*p), 4)
    verifyEqual(testCase, numel((randn(2)+randn(2)*p)), 4)
end

function testPBinterp(testCase)
    Grid = {[0 1], [ 1 2 3]};
    V = [ 0 0 0; 0 1 0];
    pb = pbinterp(Grid, V);
    pb2 = pbinterp(Grid, V+2);
    tm3 = timemap(randi(201, 1, 10), ...
        'ct', ...
        'Name', {'a', 'c'}, ...
        'Unit', {'-', 'm'}, ...
        'Range', {[-100, 100], [-Inf, +Inf]});

    mat = rand(4,4,2);

    pmatrix(mat(:,:,1),...
        'BasisType', 'interp', ...
        'BasisParametrization', ...
        {{Grid, V}}, ...
        'SchedulingTimeMap', tm3);

    P2 = pmatrix(mat,...
        'BasisType', 'interp', ...
        'BasisParametrization', ...
        {{Grid, V}, {Grid, V+2}}, ...
        'SchedulingTimeMap', tm3);

    P3 = pmatrix(mat, ...
        {pb, pb2}, ...
        'SchedulingTimeMap', tm3);

    verifyEqual(testCase,P2,P3)
end
