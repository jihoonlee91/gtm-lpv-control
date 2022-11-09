function tests = arxTest
%ARXTEST Test identification using ARX
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
function testArx(testCase)
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
    sys.Ts = Ts;                   % Sampling time
    sys.InputName = {'Voltage'};   % Naming of the signals
    sys.OutputName = {'Position'};
    sys.NoiseVariance = 0.00005^2;    % Specify noise variance


    %% Create data sets

    N=2000;
    p_mag=0.25;

    % Estimation data
    u=idinput(N,'rgs');
    p=(1-p_mag)+p_mag*idinput(N,'rbs',[0,0.05]);

    [y,~,~,~] = lsim(sys,p,u);       % Generate output data

    data_est = lpviddata(y,p,u,Ts);      % Create an LPV data object

    % Create a validation data set
    uv=idinput(N,'rgs');
    pv=(1-p_mag)+p_mag*idinput(N,'rbs',[0,0.05]);
    [yv,~,~,~] = lsim(sys,pv,uv); 

    %% LPV-ARX identification

    % Create spcification of the model structure
    th=preal('Theta','dt');
    th1=pshift(th,-1);
    th2=pshift(th,-2);

    % dimension of the coefficents and their functional dependency creates a
    % template for the model structure. The individual parameter values do not
    % matter
    am1=1+th+th1+th2;
    am2=am1;
    bm0=am1;
    bm1=am1;
    bm2=am1;

    % perform ARX identification with the specified model structure
    options = lpvarxOptions('Display','off');
    sysest = lpvarx(data_est, lpvidpoly({1, am1,am2},{bm0,bm1,bm2}),options);

    % Simulate sysest with no noise on the validation data
    ys = lsim(sysest,pv,uv,[],zeros(N,1));        
    fit = bfr(yv,ys);

    verifyEqual(testCase, fit, 90.38, 'AbsTol', 0.01);
end

function testArxNonFree(testCase)
    % Test whether 0 is set as non-free by default
    pTemplate = preal('p', 'ct') * eye(1);
    sys = lpvidpoly({1, pTemplate, 0, pTemplate}, {0, pTemplate});
   
    % What should be free is free
    verifyTrue(testCase, all(sys.A.Free{2}, 'all') && ...
        all(sys.A.Free{4}, 'all') && ...
        all(sys.B.Free{2}, 'all'));
    % What should be non-free is non-free
    verifyTrue(testCase, all(~sys.A.Free{3}, 'all') && all(~sys.B.Free{1}, 'all'));
end

function testArxMimo(testCase)
    rng(42);
    p = preal('p', 'dt');
    Ts = 1;
    ny = 2;
    NoiseVariance = 0.01 * eye(ny);
    A = {eye(ny)};
    B = {eye(ny) * p};
    sys = lpvidpoly(A, B, [], [], [], NoiseVariance, Ts, ...
        'InputDelay', 0);
    sysLti = idpoly({1, 0; 0, 1}, {1, 0; 0, 1}, [], [], [], NoiseVariance, Ts, ...
        'InputDelay', 0);
    N = 100;
    p = ones(N, sys.Np);
    u = randn(N, sys.Nu);
    y = lsim(sys, p, u, [], zeros(N, sys.Ny));
    yLti = lsim(sysLti, u, []);
    verifyTrue(testCase, norm(y - yLti) < 1E-4);
end

function testArxInputDelay(testCase)
    % Test ARX identification with InputDelay
    p = preal('p', 'dt');
    pFrozen = 0.5;
    inputDelay = 3;

    syslti = idpoly(1, [pFrozen, 0.2 * pFrozen], [], [], [], 0);
    sys = lpvidpoly(1, {p, 0.2 * p}, [], [], [], 0);

    syslti.InputDelay = inputDelay;
    sys.InputDelay = inputDelay;

    N = 20;
    u = randn(N, sys.Nu);
    p = pFrozen * ones(N, sys.Np);
    ylti = lsim(syslti, u);
    y = lsim(sys, p, u);
    % Ensure the LPV and LTI systems have identical response
    verifyTrue(testCase, norm(y - ylti) < 1E-10);

    data = iddata(y, u, 1);
    lpvdata = lpviddata(y, p, u);

    %% Identify system
    sysestlti = arx(data, [0, 2, inputDelay]);
    sysest = lpvarx(lpvdata, sys);

    yestlti = lsim(sysestlti, u);
    yest = lsim(sysest, p, u);
    verifyTrue(testCase, norm(yestlti - yest) < 1E-10);
end
