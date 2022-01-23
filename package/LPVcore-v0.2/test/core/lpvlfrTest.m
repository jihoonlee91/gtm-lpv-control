function tests = lpvlfrTest
%LPVSSTEST Test behavior of LPVLFR
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    rng(42); % make results reproducible
    
    G1 = drss(4, 4, 4);
    G2 = drss(4, 4, 4);

    p = preal('p', 'dt');
    q = preal('q', 'dt');
    D1 = p * eye(2);
    D2 = q * (rand(2) - 0.5);

    sys1 = lpvlfr(D1, G1);
    sys1.InputName = {'u11', 'u12'};
    sys1.OutputName = {'y11', 'y12'};
    sys2 = lpvlfr(D2, G2);
    sys2.InputName = {'u21', 'u22'};
    sys2.OutputName = {'y21', 'y22'};
    
    testCase.TestData.sys1 = sys1;
    testCase.TestData.sys2 = sys2;
    
    % Data for simulation
    N = 10;
    testCase.TestData.u1 = randn(N, sys1.Nu);
    testCase.TestData.u2 = randn(N, sys2.Nu);
    testCase.TestData.p1 = randn(N, sys1.Np);
    testCase.TestData.p2 = randn(N, sys2.Np);
end

%% Test functions
function testCreation(testCase)
    % Test creation using mixture of static, continuous time and discrete time
    % components.
    s = 1;
    c = preal('c', 'ct');
    d = preal('d', 'dt');
    
    verifyTrue(testCase, lpvlfr(c, s).Ts == 0);
    verifyTrue(testCase, lpvlfr(d, s).Ts == -1);
    verifyTrue(testCase, lpvlfr([], 1).Ts == 0);
    
    % Can convert from zpk and tf models
    sys = drss(3, 2, 2);
    sys_tf = lpvlfr(tf(sys));
    sys_zpk = lpvlfr(zpk(sys));
    
    verifyTrue(testCase, sys_tf.Ts == -1 && sys_zpk.Ts == -1);
end

function testDisp(testCase)
    disp(testCase.TestData.sys1);
    disp(testCase.TestData.sys2);
end

function testPlus(testCase)
    sys1 = testCase.TestData.sys1;
    sys2 = testCase.TestData.sys2;
    
    np = sys1.Np;
    nq = sys2.Np;
    nu = sys1.Nu;
    N = 100;

    %% Evaluate PLUS
    sys = sys1 + sys2;
    
    % IO size doesn't change
    verifyTrue(testCase, sys.Nu == sys1.Nu && sys1.Nu == sys2.Nu && ...
        sys.Ny == sys1.Ny && sys1.Ny == sys2.Ny);
    
    pData = rand(N, np) - 0.5;
    qData = rand(N, nq) - 0.5;
    uData = randn(N, nu);
    
    y1 = lsim(sys1, pData, uData);
    y2 = lsim(sys2, qData, uData);
    y = lsim(sys, [pData, qData], uData);
    
    verifyTrue(testCase, sys.SchedulingTimeMap.Name{2} == 'q');
    verifyTrue(testCase, max(max(abs(y - (y1 + y2)))) < 1E-10);
end

function testLSim(testCase)
    G = drss(3, 3, 4);
    p = preal('p', 'dt');
    Delta = p * randn(2);
    sys = lpvlfr(Delta, G);
    
    pFrozen = 0.2;
    sysLocal = extractLocal(sys, pFrozen);
    
    nu = sys.Nu;
    N = 10;
    uData = randn(N, nu);
    t = linspace(0, N-1, N)';
    y = lsim(lft(freeze(Delta, pFrozen), G), uData, t);
    yLocal = lsim(sysLocal, uData);
    
    verifyTrue(testCase, max(max(abs(y - yLocal))) < 1E-10);
    
    % CT simulation
    rng(3);
    G = rss(2, 2, 2);
    Delta = preal('p', 'ct');
    Q = lpvlfr(Delta, G);
    QLocal = extractLocal(Q, 0);
    assert(isstable(QLocal));
    u = randn(N, Q.Nu);
    y = lsim(Q, zeros(N, 1), u, t);
    yLocal = lsim(QLocal, u, t);
    verifyTrue(testCase, norm(y - yLocal) < 1E-2);
end

function testLft(testCase)
    p = preal('p', 'dt');
    q = preal('q', 'dt');
    nx = 4;
    nu = 4;
    ny = 4;
    G1 = ss(randn(nx), randn(nx, nu), randn(ny, nx), randn(ny, nu), -1);
    G2 = ss(randn(nx), randn(nx, nu), randn(ny, nx), randn(ny, nu), -1);
    sys1 = lpvlfr(p, G1);
    sys2 = lpvlfr(q, G2);
    sys3 = drss(2, 3, 3);
    sys3 = lpvss(sys3.A+p,sys3.B,sys3.C,sys3.D, -1);
    
    % LFT of LPVLFR with LPVLFR
    sys = lft(sys1, sys2, 1, 1);
    pFrozen = 0.8;
    sysLocal = extractLocal(sys, pFrozen * ones(1, sys.Np));
    sysLocal1 = extractLocal(sys1, pFrozen * ones(1, sys1.Np));
    sysLocal2 = extractLocal(sys2, pFrozen * ones(1, sys2.Np));
    sysLocalTilde = lft(sysLocal1, sysLocal2, 1, 1);
    
    verifyTrue(testCase, sscompare(sysLocal, sysLocalTilde));
    
    % LFT of LPVSS with LPVLFR and reverse yields LPVLFR
    verifyTrue(testCase, ...
        isa(lft(sys2, sys3, 1, 1), 'lpvlfr'));
    verifyTrue(testCase, ...
        isa(lft(sys3, sys2, 1, 1), 'lpvlfr'));
    
    % LFT of LPVLFR with constant
    C = randn(2, 2);
    sys = lft(sys1, C, 1, 1);
    sysLocal = extractLocal(sys, pFrozen * ones(1, sys.Np));
    sysLocalTilde = lft(sysLocal1, C, 1, 1);
    
    verifyTrue(testCase, sscompare(sysLocal, sysLocalTilde));
    
    % LFT of constant with LPVLFR
    sys = lft(C, sys1, 1, 1);
    sysLocal = extractLocal(sys, pFrozen * ones(1, sys.Np));
    sysLocalTilde = lft(C, sysLocal1, 1, 1);
    
    verifyTrue(testCase, sscompare(sysLocal, sysLocalTilde));
end

function testLpvss2lfr(testCase)
    % Test conversion from lpvss to lpvlfr (and back)
    p = preal('p', 'dt');
    q = preal('q', 'dt');
    A = 0.01 * (randn(4) + randn(4) * p) * (randn(4) + randn(4) * q);
    B = randn(4, 1) * p + randn(4, 1) * q + randn(4, 1);
    C = B';
    D = zeros(size(C, 1), size(B, 2));
    sysss = lpvss(A, B, C, D);
    syslfr = lpvlfr(sysss);

    % IO response is identical
    N = 5;
    uData = randn(N, sysss.Nu);
    pData = randn(N, sysss.Np);
    
    y = lsim(sysss, pData, uData);
    ytilde = lsim(syslfr, pData, uData);
    
    verifyTrue(testCase, max(max(abs(y - ytilde))) < 1E-10);
    
    % Converting back to lpvss gives the same system
    syssstilde = lpvss(syslfr);
    
    verifyTrue(testCase, sysss.A == syssstilde.A && sysss.B == syssstilde.B && ...
        sysss.C == syssstilde.C && sysss.D == syssstilde.D);
end

function testCat(testCase)
    % TESTCAT
    %
    %   Test concatenation of LPV-LFR systems by doing an LSIM on [sys1,
    %   sys2] and [sys1; sys2] with random data and ensuring the reponses
    %   are concatenated as well. 
    %    
    %   Note: LSIM should be short, as sys1, sys2 can be unstable! 5
    %   samples or so are enough.
    %
    sys = testCase.TestData.sys1;
    
    sysv = [sys; sys];
    sysh = [sys, sys, sys];
    
    N = 5;
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

function testBlkdiag(testCase)
    sys1 = testCase.TestData.sys1;
    sys2 = testCase.TestData.sys2;
    
    sys = blkdiag(sys1, sys2, sys1, sys2);
    
    y1 = lsim(sys1, testCase.TestData.p1, testCase.TestData.u1);
    y2 = lsim(sys2, testCase.TestData.p2, testCase.TestData.u2);
    y = lsim(sys, [testCase.TestData.p1, testCase.TestData.p2], ...
        [testCase.TestData.u1, testCase.TestData.u2, ...
        testCase.TestData.u1, testCase.TestData.u2]);
    
    verifyTrue(testCase, sys.Nu == 2 * (sys1.Nu + sys2.Nu) && ...
        sys.Ny == 2 * (sys1.Ny + sys2.Ny));
    verifyTrue(testCase, max(max(abs(y - [y1, y2, y1, y2]))) < 1E-10);
    
    %% Test blkdiag for LPV-LFR objects with empty Delta block
    Delta = [];
    G = drss(2, 2, 2);
    sys = lpvlfr(Delta, G);
    sysc = blkdiag(sys, sys);
    
    verifyTrue(testCase, sysc.Ny == 2 * sys.Ny && sysc.Nu == 2 * sys.Nu && ...
        sysc.Np == 0);
    
    sysc = blkdiag(sys, sys1);
    
    verifyTrue(testCase, sysc.Ny == sys.Ny + sys1.Ny && ...
        sysc.Np == sys1.Np);
end

function testSubsref(testCase)
    sys1 = testCase.TestData.sys1;
    
    syst = sys1(1, 1);
    
    verifyTrue(testCase, syst.Nu == 1 && syst.Ny == 1 && ...
        syst.Np == sys1.Np && ...
        strcmp(syst.InputName{1}, sys1.InputName{1}) && ...
        strcmp(syst.OutputName{1}, sys1.OutputName{1}));
    verifyTrue(testCase, sys1.Delta == syst.Delta && ...
        all(abs(syst.Bu - sys1.Bu(:, 1)) < eps) && ...
        all(abs(syst.Cy - sys1.Cy(1, :)) < eps));
end

function testFeedback(testCase)
    C = pid(2,1); 
    C.InputName = {'B'};
    C.OutputName = {'A'};
    G = tf(1, [1 2 1]);     
    sys = lpvlfr([],G);
    sys.InputName = {'A'};
    sys.OutputName = {'B'};
    % 1. feedback(sys1,sys2)
    % use lfrfeedback and connect
    sysFeedback1 = extractLocal(feedback(sys,ss(C)),[]);
    % use standard feedback
    sysCheck1 = feedback(extractLocal(sys,[]),C);   
    
    % 2. feedback(sys1,sys2,sign)
    % use lfrfeedback and connect
    sysFeedback2 = extractLocal(feedback(sys,ss(C),1),[]);
    % use standard feedback
    sysCheck2 = feedback(extractLocal(sys,[]),C,1);
    
     % 2. feedback(sys1,sys2,'name')
    % use lfrfeedback and connect
    sysFeedback3 = extractLocal(feedback(sys,ss(C),'name'),[]);
    % use standard feedback
    sysCheck3 = feedback(extractLocal(sys,[]),C,'name');
    
    
    % 4. feedback(sys1,sys2,feedin,feedout, sign) MIMO    
    sysFeedback4 = extractLocal(feedback(sys,ss(C), 1, 1),[]);
    % use standard feedback
    sysCheck4 = feedback(extractLocal(sys,[]),C, 1, 1);
    
    % verify results
    verifyTrue(testCase,sscompare(sysFeedback1,sysCheck1) == 1);
    verifyTrue(testCase,sscompare(sysFeedback2,sysCheck2) == 1);
    verifyTrue(testCase,sscompare(sysFeedback3,sysCheck3) == 1);
    verifyTrue(testCase,sscompare(sysFeedback4,sysCheck4) == 1);
end
    
function testConnect(testCase)
    %% With empty Delta block
    rng(3);  % ensure stable connection
    G = drss(2, 2, 2);
    G.D = randn(size(G.D));
    % I - D should be non-singular, where D is the direct feedthrough from
    % u --> y
    assert(1 - G.D(1, 1) ~= 0)
    G.InputName = {'u', 'c'};
    G.OutputName = {'y', 'c'};
    
    sys = lpvlfr([], G, 'InputName', {'u', 'c'}, 'OutputName', {'y', 'c'});
    sysc = connect(sys, 'u', 'y');
    syscLocal = connect(G, 'u', 'y');
    
    % Compare
    verifyTrue(testCase, abs(norm(extractLocal(sysc, [])) - norm(syscLocal)) < 1E-10);
    
    %% With non-empty Delta block
    sys = testCase.TestData.sys1;
    sys.InputName = {'u', 'c'};
    sys.OutputName = {'y', 'c'};
    p = 0.1 * ones(1, sys.Np);
    
    % Connect then freeze
    sysc = connect(sys, 'u', 'y');
    syscLocal = extractLocal(sysc, p);
    % Freeze then connect
    syscLocalTilde = connect(extractLocal(sys, p), 'u', 'y');
    
    verifyTrue(testCase, abs(norm(syscLocal) - norm(syscLocalTilde)) < 1E-10);
end

function testTs(testCase)
    TsVec = [-1, 0, 1];
    for i=1:numel(TsVec)
        Ts = TsVec(i);
        % Test concatenation of LPV-SS
        P = lpvss(1, 1, 1, 1, Ts);
        verifyTrue(testCase, blkdiag(1, P).Ts == Ts);
        verifyTrue(testCase, blkdiag(P, 1).Ts == Ts);
        verifyTrue(testCase, blkdiag(1, P, 1).Ts == Ts);
        
        % Test concatenation of LPV-LFR
        G = drss(2, 1, 1);
        G.Ts = Ts;
        Q = lpvlfr([], G);
        verifyTrue(testCase, Q.Ts == Ts);
        verifyTrue(testCase, blkdiag(P, Q).Ts == Ts);
        verifyTrue(testCase, blkdiag(1, P, 1, Q).Ts == Ts);
        verifyTrue(testCase, blkdiag(Q, 1, P).Ts == Ts);
    end
end

function testProperties(testCase)
    inputDelay = [1; 1];
    inputName = {'u1', 'u2'};
    outputName = {'y1', 'y2'};
    inputUnit = {'cm', 'm'};
    outputUnit = {'m/s', 'J'};
    stateName = {'power', 'viscosity'};
    stateUnit = {'W', 'Pa s'};
    nu = numel(inputName);
    ny = numel(outputName);
    nx = numel(stateName);
    p = preal('p', 'ct');
    Delta = p;
    G = rss(nx, ny + size(Delta, 1), nu + size(Delta, 2));
    sys = lpvlfr(Delta, G, ...
        'InputDelay', inputDelay, ...
        'InputName', inputName, ...
        'OutputName', outputName, ...
        'InputUnit', inputUnit, ...
        'OutputUnit', outputUnit, ...
        'StateName', stateName, ...
        'StateUnit', stateUnit);
    syslti = ss(eye(nx), ones(nx, nu), ...
        ones(ny, nx), ones(ny, nu), sys.Ts, ...
        'InputDelay', inputDelay, ...
        'InputName', inputName, ...
        'OutputName', outputName, ...
        'InputUnit', inputUnit, ...
        'OutputUnit', outputUnit, ...
        'StateName', stateName, ...
        'StateUnit', stateUnit);
    
    verifyTrue(testCase, areSharedPropsEqual(sys, syslti))
    verifyTrue(testCase, ...
        areSharedPropsEqual(sys^2, syslti^2));
    verifyTrue(testCase, ...
        areSharedPropsEqual([sys, sys; sys, sys], ...
        [syslti, syslti; syslti, syslti]));
    verifyTrue(testCase, ...
        areSharedPropsEqual(sys + sys, syslti + syslti));
end