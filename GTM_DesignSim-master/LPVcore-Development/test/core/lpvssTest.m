function tests = lpvssTest
%LPVSSTEST Test behavior of LPVSS
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
    tm1 = timemap(randi(201, 1, 10) - 101, ...
        'dt', ...
        'Name', {'a', 'b'}, ...
        'Unit', {'-', 'cm'}, ...
        'Range', {[-100, 100], [0, 1]});
    tm2 = timemap(randi(201, 1, 10) - 101, ...
        'dt', ...
        'Name', {'a', 'c'}, ...
        'Unit', {'-', 'm'}, ...
        'Range', {[-100, 100], [-Inf, +Inf]});

    testCase.TestData.P1 = pmatrix(0.005 * rand(4, 4, 2), ...
        'BasisType', 'affine', ...
        'SchedulingTimeMap', tm1);
    testCase.TestData.P2 = pmatrix(randn(4, 4, 2), ...
        'BasisType', 'affine', ...
        'SchedulingTimeMap', tm2);

    testCase.TestData.sys = lpvss(testCase.TestData.P1, ...
        testCase.TestData.P2, testCase.TestData.P1, ...
        testCase.TestData.P2, 0.1);
    
    p = preal('p', 'dt');
    A = randn(2) + p * randn(2);
    B = randn(2) + p * randn(2);
    C = randn(2) + p * randn(2);
    D = zeros(2);
    testCase.TestData.sys2 = lpvss(A, B, C, D, 0.1);
end

%% Test functions
function testCreation(testCase)
    % Test creation using mixture of static, continuous time and discrete time
    % components.
    s = 1;
    c = preal('c', 'ct');
    d = preal('d', 'dt');
    Ts = 11;
    
    verifyTrue(testCase, lpvss(s, s, s, s).Ts == 0);
    verifyTrue(testCase, lpvss(s, s, s, d).Ts == -1);
    verifyTrue(testCase, lpvss(s, s, s, c).Ts == 0);
    verifyTrue(testCase, lpvss(s).Ts == 0);
    verifyTrue(testCase, lpvss(s, s, s, s, Ts).Ts == Ts);
    verifyTrue(testCase, lpvss(s, d, d, s, Ts).Ts == Ts);
    
    verifyError(testCase, @() lpvss(c, c, c, d).Ts, '');
    
    % D can be left empty
    sys = lpvss(eye(2), [1; 1], [1, 1], [], 0);
    syslti = ss(eye(2), [1; 1], [1, 1], [], 0);
    verifyTrue(testCase, areSharedPropsEqual(sys, syslti));
    
    % Can convert from zpk and tf models
    sys = drss(3, 2, 2);
    sys_tf = lpvss(tf(sys));
    sys_zpk = lpvss(zpk(sys));
    verifyTrue(testCase, sys_tf.Ts == -1 && sys_zpk.Ts == -1);

    % Signal properties match
    G = rss(2, 2, 1);
    G.InputName = 'u';
    G.Outputname = {'y1', 'y2'};
    G.StateName = {'position', 'velocity'};
    Glpv = lpvss(G);
    verifyTrue(testCase, LPVcore.doSignalPropertiesMatch(G, Glpv, 'full'));

    % B, C can be empty (see #348)
    P = lpvss(0,[],2,3);
    Pl = ss(0,[],2,3);
    verifyEqual(testCase, extractLocal(P, []), Pl);
    
    % Can also assign B, C, D to empty
    P.B = []; P.C = []; P.D = [];
    Pl.B = []; Pl.C = []; Pl.D = [];
    verifyEqual(testCase, extractLocal(P, []), Pl);
    
    P = lpvss(2,[],[],3);
    Pl = ss(2,[],[],3);
    verifyEqual(testCase, extractLocal(P, []), Pl);

    % B, C, D must have compatible size
    verifyError(testCase, @() lpvss(-1, -1, 0, [2, 1]), '');
    verifyError(testCase, @() lpvss([], [], 0, 2), '');
    verifyError(testCase, @() lpvss(-1, -1, [0; 1], 2), '');
end

function testDisp(testCase)
    disp(testCase.TestData.sys);
end

function testSize(testCase)
    sys = testCase.TestData.sys;
    P1 = testCase.TestData.P1;
    
    verifyTrue(testCase, all([size(P1, 1), size(P1, 2)] == size(sys)));
end

function testLSim(testCase)
    sys = testCase.TestData.sys;
    
    N = 10;
    nu = sys.Nu;
    np = sys.Np;
    ny = sys.Ny;
    
    [y, t] = lsim(sys, randn(N, np), randn(N, nu));
    
    verifyTrue(testCase, all(size(y) == [N, ny]) && ...
        numel(t) == N);
    
    % MIMO DT simulation compared with LTI
    nx = 2; ny = 2; nu = 2;
    Ts = 1.5;
    x0 = randn(nx, 1);

    A = rand(nx);
    B = ones(nx, nu);
    C = ones(ny, nx);

    sys = lpvss(A, B, C, [], Ts);
    syslti = ss(A, B, C, [], Ts);

    N = 10;
    u = randn(N, sys.Nu);
    p = zeros(N, sys.Np);

    [y, t, x] = lsim(sys, p, u, [], x0);
    [y2, t2, x2] = lsim(syslti, u, [], x0);
    
    verifyTrue(testCase, norm(y - y2) < 1E-10 && ...
        norm(x - x2) < 1E-10 && ...
        norm(t - t2) < 1E-10);
    
    % MIMO CT simulation compared with LTI
    Ts = 0;
    p = preal('p', 'ct');
    
    sys = lpvss(A, B, C, p * ones(ny, nu), Ts);
    syslti = ss(A, B, C, [], Ts);
    
    t = linspace(0, 1, N);
    [y, t, x] = lsim(sys, zeros(N, sys.Np), u, t, x0, 'Solver', @ode23);
    [y2, t2, x2] = lsim(syslti, u, t, x0, 'foh');  % LPVcore uses FOH for CT sim
    
    verifyTrue(testCase, mean(bfr(y, y2)) > 90 && ...
        norm(t - t2) < 1E-10 && ...
        isequal(size(x), size(x2)));
end

function testPlus(testCase)
    sys = testCase.TestData.sys;
    sysPlus = sys + sys;
    
    N = 10;
    nu = sys.Nu;
    np = sys.Np;
    
    p = randn(N, np);
    u = randn(N, nu);
    
    [y, ~] = lsim(sys, p, u);
    [yPlus, ~] = lsim(sysPlus, p, u);
    
    verifyTrue(testCase, all(all(abs(yPlus - 2 * y) <= 1E-10)));
end

function testMTimes(testCase)
    sys = testCase.TestData.sys;
    
    N = 10;
    nu = sys.Nu;
    np = sys.Np;
    
    u = randn(N, nu);
    p = randn(N, np);
    
    % LPVSS * LPVSS
    y = lsim(sys, p, u);
    yy = lsim(sys, p, y);
    yycheck = lsim(sys * sys, p, u);
    
    verifyTrue(testCase, all(abs(yy - yycheck) < 1E-10, 'all'));
    
    % LPVSS * scalar
    y2 = lsim(sys * 2, p, u);
    verifyTrue(testCase, all(abs(2 * y - y2) < 1E-10, 'all'));
    y2 = lsim(2 * sys, p, u);
    verifyTrue(testCase, all(abs(2 * y - y2) < 1E-10, 'all'));
    
    %% Test with two random systems
    nx = 2;                 % State dimension of systems

    %% Create random system
    ny = 1;                 % # outputs of G
    nu = 1;                 % # inputs of G

    Ag = randn(nx,nx);
    Bg = randn(nx,nu);
    Cg = randn(ny,nx);
    Dg = randn(ny,nu);

    G = lpvss(Ag,Bg,Cg,Dg); %% LPVSS
    Glti = ss(Ag,Bg,Cg,Dg); %% SS

    %% Create other random system 
    nm = 1;                 % # outputs of M (dimension error if nm>nu)

    Am = randn(nx,nx);
    Bm = randn(nx,ny);
    Cm = randn(nm,nx);
    Dm = randn(nm,ny);

    M = lpvss(Am,Bm,Cm,Dm); %% LPVSS
    Mlti = ss(Am,Bm,Cm,Dm); %% SS

    %% Compute interconnection
    Plti = Mlti*Glti;       % SS
    P = M*G;                % LPVSS

    verifyTrue(testCase, ...
        norm(Plti.A - P.A.matrices) < 1E-10 && ...
        norm(Plti.B - P.B.matrices) < 1E-10 && ...
        norm(Plti.C - P.C.matrices) < 1E-10 && ...
        norm(Plti.D - P.D.matrices) < 1E-10);
end

function testMPower(testCase)
    sys = testCase.TestData.sys;
    
    sys0 = sys^0;
    sys1 = sys^1;
    sys2 = sys^2;
    sys2_ = sys * sys;
    
    verifyTrue(testCase, sys0.A == zeros(sys.Nx));
    verifyTrue(testCase, sys1.A == sys.A);
    verifyTrue(testCase, sys2.A == sys2_.A);
end

function testCat(testCase)
    sys = testCase.TestData.sys;
    
    sysv = [sys; sys];
    sysh = [sys, sys, sys];
    
    N = 100;
    nu = sys.Nu;
    np = sys.Np;
    ny = sys.Ny;
    
    u = randn(N, nu);
    p = randn(N, np);
    
    y = lsim(sys, p, u);
    yv = lsim(sysv, p, u);
    yh = lsim(sysh, p, [u, u, u]);
    
    verifyTrue(testCase, all(abs(y - yv(:, 1:ny)) < 1E-10, 'all'));
    verifyTrue(testCase, all(abs(y - yv(:, ny+1:end)) < 1E-10, 'all'));
    verifyTrue(testCase, all(abs(3 * y - yh) < 1E-10, 'all'));
end

function testExtractLocal(testCase)
    sys = testCase.TestData.sys;
    p = rand(1, sys.Np);
    
    sysLocal = extractLocal(sys, p);
    ALocal = freeze(sys.A, p);
    BLocal = freeze(sys.B, p);
    CLocal = freeze(sys.C, p);
    DLocal = freeze(sys.D, p);
    
    verifyTrue(testCase, sscompare(sysLocal, ss(ALocal, BLocal, CLocal, DLocal)));
end

function testLft(testCase)
    sys1 = testCase.TestData.sys;
    sys1.D = zeros(size(sys1.D, 1));
    sys2 = testCase.TestData.sys2;
    pFrozen = 0.8;
    
    % sysLocal: LFT, then freeze
    sys = lft(sys2, sys1);
    sysLocal = extractLocal(sys, pFrozen * ones(1, sys.Np));
    
    % sysLocal2: Freeze, then LFT
    sys1Local = extractLocal(sys1, pFrozen * ones(1, sys1.Np));
    sys2Local = extractLocal(sys2, pFrozen * ones(1, sys2.Np));
    sysLocal2 = lft(sys2Local, sys1Local);
    
    % sysLocal == sysLocal2
    verifyTrue(testCase, sscompare(sysLocal, sysLocal2));
    
    % LFT remains LPV-SS model for diffferent combinations...
    p = preal('p', 'dt', 'Range',[-1,1]);

    sys1 = drss(2, 1, 1);
    sys1 = lpvss(sys1.A+p,sys1.B,sys1.C,sys1.D, 1);
    
    sys2 = drss(2, 3, 3);
    sys2 = lpvss(sys2.A+p,sys2.B,sys2.C,sys2.D, 1);
    
    % ...LPV-SS with LPV-SS
    sys = lft(sys1, sys2);
    verifyTrue(testCase, isa(sys, 'lpvss'));
    
    % ...Constant with LPV-SS
    sys = lft(1, sys2);
    verifyTrue(testCase, isa(sys, 'lpvss'));
    
    % ...LPV-SS with constant
    sys = lft(sys1, randn(3));
    verifyTrue(testCase, isa(sys, 'lpvss'));
end

function testProperties(testCase)
    inputDelay = [1; 1];
    inputName = {'u1', 'u2'};
    outputName = {'y1', 'y2'};
    inputUnit = {'cm', 'm'};
    outputUnit = {'m/s', 'J'};
    nu = numel(inputName);
    ny = numel(outputName);
    p = preal('p', 'ct');
    Ts = 0;
    sys = lpvss(p, ones(1, nu), ...
        ones(ny, 1), ones(ny, nu), Ts, ...
        'InputDelay', inputDelay, ...
        'InputName', inputName, ...
        'OutputName', outputName, ...
        'InputUnit', inputUnit, ...
        'OutputUnit', outputUnit);
    syslti = ss(1, ones(1, nu), ...
        ones(ny, 1), ones(ny, nu), Ts, ...
        'InputDelay', inputDelay, ...
        'InputName', inputName, ...
        'OutputName', outputName, ...
        'InputUnit', inputUnit, ...
        'OutputUnit', outputUnit);
    
    verifyTrue(testCase, areSharedPropsEqual(sys, syslti))
    verifyTrue(testCase, ...
        areSharedPropsEqual(sys^2, syslti^2));
    verifyTrue(testCase, ...
        areSharedPropsEqual([sys, sys; sys, sys], ...
        [syslti, syslti; syslti, syslti]));
    verifyTrue(testCase, ...
        areSharedPropsEqual(sys + sys, syslti + syslti));
end

