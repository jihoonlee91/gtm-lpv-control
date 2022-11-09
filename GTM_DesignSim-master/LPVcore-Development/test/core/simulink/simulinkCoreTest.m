function tests = simulinkCoreTest
%SIMULINKTEST Test simulink blocks in the general library.
    tests = functiontests(localfunctions);
end

%% Continuous time LPVSS block test
function testLpvssCt(testCase)
    % LTI case
    nx = 5; nu = 1; ny = 1;
    x0 = randn(nx,1);

    P1l = rss(nx,ny,nu);
    P1 = lpvss(P1l);

    % LPVSS case
    % xd(1) = x(2);
    % xd(2) = -x(1)-20*p(1)*x(1)-p(2)^2*x(2)+u;
    p1 = preal('p1','ct');
    p2 = preal('p2','ct');

    Ag = [0 1;-1-20*p1 -p2^2];
    Bg = [0;1];
    Cg = eye(2);
    Dg = zeros(2,1);

    P2 = lpvss(Ag,Bg,Cg,Dg);
    
    % LPVLFR case
    A0 = diag([-1 -2]);
    Bw = [1 3;2 6];
    Bu = [1;1];
    Cz = [1 3];
    Cy = [2 1];
    Dzw = [-1 -1];
    Dzu = 3;
    Dyw = [2 3];
    Dyu = 5;
    
    G = ss(A0,[Bw,Bu],[Cz;Cy],[Dzw,Dzu;Dyw,Dyu]);
    Delta = [p1;p2];
    P3 = lpvlfr(Delta,G);

    % Simulate
    simIn = Simulink.SimulationInput('simLpvssCtTest');
    simIn = setVariable(simIn,'x0',x0);
    simIn = setVariable(simIn,'P1l',P1l);
    simIn = setVariable(simIn,'P1',P1);
    simIn = setVariable(simIn,'P2',P2);
    simIn = setVariable(simIn,'P3',P3);
    out = sim(simIn);
    
    % Check    
    checkCt = [ norm(out.P1-out.P1r)<=eps;
                norm(out.P2-out.P2r)<=5*eps;
                norm(out.P3-out.P3r)<=1e-10; ];
    
    verifyTrue(testCase, all(checkCt,'all'));   
end

%% Discrete time LPVSS block test
function testLpvssDt(testCase)
    Ts = 0.1;

    % LTI case
    nx = 2; nu = 1; ny = 1;
    x0 = randn(nx,1);

    P1l = drss(nx,ny,nu);P1l.Ts = Ts;
    P1 = lpvss(P1l);

    % LPV case
    % xp(1) = 0.8*x(2);
    % xp(2) = 0.1*p(1)*x(1)+0.5*x(2)*p(2)+u;
    p1 = preal('p1','dt');
    p2 = preal('p2','dt');

    Ag = [0 .8;0.1*p1 0.5*p2];
    Bg = [0;1];
    Cg = eye(2);
    Dg = zeros(2,1);

    P2 = lpvss(Ag,Bg,Cg,Dg,Ts);
    
    % LPVLFR case
    A0 = diag([.5 -.2]);
    Bw = [1 3;2 6];
    Bu = [1;1];
    Cz = [1 3];
    Cy = [2 1];
    Dzw = [-1 -1];
    Dzu = 3;
    Dyw = [2 3];
    Dyu = 5;
    
    G = ss(A0,[Bw,Bu],[Cz;Cy],[Dzw,Dzu;Dyw,Dyu],Ts);
    Delta = [p1;p2];
    P3 = lpvlfr(Delta,G);

    % Simulate
    simIn = Simulink.SimulationInput('simLpvssDtTest');
    simIn = setVariable(simIn,'Ts',Ts);
    simIn = setVariable(simIn,'x0',x0);
    simIn = setVariable(simIn,'P1l',P1l);
    simIn = setVariable(simIn,'P1',P1);
    simIn = setVariable(simIn,'P2',P2);
    simIn = setVariable(simIn,'P3',P3);
    out = sim(simIn);

    % Check
    checkDt = [ norm(out.P1-out.P1r)<=eps;
                norm(out.P2-out.P2r)<=eps;
                norm(out.P3-out.P3r)<=1e-11; ];
    
    verifyTrue(testCase, all(checkDt));   
end

%% Parameter-varying matrix block test
function testPmatrix(testCase)
    p1 = preal('p1','ct');
    p2 = preal('p2','ct');
    p3 = preal('p3','ct');

    M = @(p1,p2,p3) [p1,2*p2,3*p2;p2.*p3,p1-3,p3;p1.*p3,p2.*p1,p3];  

    Mp = M(p1,p2,p3);

    % Simulate
    simIn = Simulink.SimulationInput('simPmatrixTest');
    simIn = setVariable(simIn,'Mp',Mp);
    out = sim(simIn);
    
    % Manually calculate matrix trajectory
    Mr = zeros(size(out.M));

    for i = 1:size(out.M,3)
        Mr(:,:,i) = M(out.p(i,1),out.p(i,2),out.p(i,3));
    end
    
    % Check
    checkPmatrix = [norm(reshape(out.M-Mr,[],1))<=eps;
                    all(squeeze(out.M0) == 0);
                    all(squeeze(out.M1) == 2)];
                
    verifyTrue(testCase, all(checkPmatrix));   
end