function tests = armaxTest
%ARMAXTEST Test identification using ARMAX
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
    rng(42);   % set random number generator seed so that results are reproducible

    %% Define model parameters
    K=53.6*10^-3;   % [Nm/A] Motor torque constant
    R=9.50;         % [Ohm] Motor resistance
    J=2.2*10^-4;    % [Nm^2] Complete disk inertia
    b=6.6*10^-5;    % [Nms/rad] Friction coefficient
    M=0.07;         % [kg] Additional mass
    l=0.042;        % [m] Mass distance from the disk center
    g=9.8;          % [m/s^2] Gravitational acceleration 
    tau=(R*J)/(R*b+K^2);       % lumped back emf constant
    Km=K/(R*b+K^2);            % limpued torque constant 

    %% Define DT motor model as an idpoly object

    Ts = 0.005;              % Sampling time
    th2= preal('Theta','dt','Dynamic',-2);
                                % Dynamic dependence, shifted back 2 samples
      
    a1=Ts/tau-2;               
    a2=1-Ts/tau+((M*g*l)/J)*(Ts^2)*th2;  

    b2=Km/tau*Ts^2;             % Coefficient of the input side

    sys = lpvidpoly({1, a1 a2},{0 0 b2}); % Create and idpolyobject => ARX model
    sys.B.Free{1} = false;
    sys.B.Free{2} = false;
    sys.Ts = Ts;                   % Sampling time
    sys.InputName = {'Voltage'};   % Naming of the signals
    sys.OutputName = {'Position'};
    sys.NoiseVariance = 0.00005^2;    % Specify noise variance
    testCase.TestData.sys = sys;

    %% Create data sets
    N=2000;
    p_mag=0.25;

    % Estimation
    u=idinput(N,'rgs');
    p=(1-p_mag)+p_mag*idinput(N,'rbs',[0,0.05]);
    [y,~,~,~] = lsim(sys,p,u);
    testCase.TestData.estData = lpviddata(y,p,u,Ts);

    % Validation
    uv=idinput(N,'rgs');
    pv=(1-p_mag)+p_mag*idinput(N,'rbs',[0,0.05]);
    [yv,~,~,~] = lsim(sys,pv,uv);       
    testCase.TestData.valData = lpviddata(yv,pv,uv,Ts);  
end

%% Test functions
function testArmax(testCase)
    rng(42);   % set random number generator seed so that results are reproducible

    %% Define system
    p1 = preal('pos', 'dt');

    a1 = 1   - 0.5*p1 - 0.3*p1*p1;
    a2 = 0.5 - 0.7*p1 - 0.5*p1*p1;

    b1 = 0.5 - 0.4*p1 + 0.1*p1*p1;
    b2 = 0.2 - 0.3*p1 - 0.2*p1*p1;

    c1 = 0.9 * p1;
    c2 = b2;
    c3 = b1;

    sys = lpvidpoly({1, a1,a2},{0,b1,b2}, {1, c1, c2, c3});
    sysArx = lpvidpoly({1, a1, a2}, {0, b1, b2});
    sys.NoiseVariance = 0.2^2;
    sysArx.NoiseVariance = sys.NoiseVariance;

    %% Generate data
    N = 4000;
    u = rand(N,1);
    p = 0.2+0.4 * sin(0.035*pi*(1:N)')+ 0.3*rand(N,1);
    [y,~,~,~,~] = lsim(sys,p,u);
    data = lpviddata(y,p,u,0.1);


    %% LPV-ARMAX identification
    sysestArx = lpvarx(data, sysArx);
    sysestArmax = lpvarmax(data, sys, ...
        lpvarmaxOptions('SearchMethod', 'pseudols', 'Display', 'off'));

    y = lsim(sys, p, u, [], zeros(N, 1));
    ys1Arx = lsim(sysestArx,p,u,[],zeros(N,1)); % Simulate sysest with no noise input
    ys1Armax = lsim(sysestArmax, p, u, [], zeros(N, 1));

    farx = LPVcore.goodnessOfFit(y, ys1Arx, 'nmse');
    farmax = LPVcore.goodnessOfFit(y, ys1Armax, 'nmse');
    
    fprintf('Fit: %d (ARMAX), %d (ARX)\n', farmax, farx);

    verifyEqual(testCase, farx, 0.074, 'AbsTol', 0.01);
    verifyEqual(testCase, farmax, 0.0195, 'AbsTol', 0.01);
end

function testInputDelay(testCase)
    rng(2);
    inputDelay = [2, 1];
    noiseVariance = 0.1;

    syslpv = lpvidpoly(1, [1, 2], {1, 0.5}, [], [], noiseVariance);
    syslti = idpoly(1, {1, 2}, [1, 0.5], [], [], noiseVariance);
    syslpv.InputDelay = inputDelay;
    syslti.InputDelay = inputDelay;

    N = 100;
    u = randn(N, syslpv.Nu);
    p = randn(N, syslpv.Np);

    ylpv = lsim(syslpv, p, u);
    data = lpviddata(ylpv, p, u);

    sysest = lpvarmax(data, randlpvidpoly(syslpv), ...
        lpvarmaxOptions('SearchMethod', 'pseudols'));

    yest = lsim(sysest, p, u);

    verifyEqual(testCase, bfr(ylpv, yest), 77.87, 'AbsTol', 1);
end