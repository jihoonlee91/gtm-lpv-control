function [out, X] = lpvnorm(P, type, varargin)
%LPVNORM computes the L1, L2 or H2 norm of a lpvss object or allows to
%check if if a lpvss object is passive.
%
%   Syntax:
%       [out, X] = lpvnorm(P, ...)
%       
%       [out, X] = lpvnorm(P, type, ...)
%
%   Inputs: 
%       P (lpvss): LPV state-space system for which the norm is to be
%           computed.
%       type (char): Type of norm to be computed, supported norms: 'l1',
%           'l2', 'h2' and 'passive'. If type is not given, 'l2' is used.
%       
%       Optional name-value pair:
%           'SolverOptions': Structure containing the YALMIP solver settings. 
%               See also <a href="matlab:help sdpsettings">sdpsettings</a>. Default value: 
%               sdpsettings('verbose',0,'solver','sdpt3');
%           'NumericalAccuracy': Numerical accuracy parameter. Used for, 
%               among other things, solving the LMIs (LMI <= NumAcc). 
%               Default value: 1e-10;
%           'L1Options': Structure containing the options for L1 analysis. 
%               See also <a href="matlab:help l1Options">l1Options</a>.
%               
%   Outputs:
%       out: Value of the norm in case of the 'l1', 'l2' or 'h2' norm or
%           true in the case of a passivity check and the system is passive.
%       X: Value of quadratic storage function matrix.      
%

%% Checks
% Check if type of norm is given
if ~exist('type','var')
    warning('No norm type given, computing L2 norm');
    type = 'l2';
% Check if norm type is supported
else
    normList = {'l1','l2','h2','passive'};
    assert(any(strcmpi(type,normList)),...
        sprintf('Unknown system norm.\nSupported system norms: l1, l2, h2, passive.'));
end
% Check if system has affine parameter dependency
assert(all([isaffine(P.A),isaffine(P.B),isaffine(P.C),isaffine(P.D)]),...
       'System is not affinely dependent on the parameter');

%% Parse optional arguments
% Default values
solOpt = sdpsettings('verbose',0,'solver','sdpt3');
numAcc = 1e-10;
l1Opt = l1Options;

% Parse
opt = inputParser;

checkStruct = @(x) isa(x,'struct');

addParameter(opt,'SolverOptions',solOpt,checkStruct);
addParameter(opt,'NumericalAccuracy',numAcc,@isnumeric);
addParameter(opt,'L1Options',l1Opt,checkStruct)

parse(opt,varargin{:});

solOpt = opt.Results.SolverOptions;
numAcc = opt.Results.NumericalAccuracy;
l1Opt = opt.Results.L1Options;

%% Variables
% determine matrix sizes
nx = P.Nx;
nu = P.Nu;
ny = P.Ny;

% check if system is CT or DT
if P.Ts == 0
    systemType = 'ct';
else
    systemType = 'dt';
end

%% Construct rolmipvariables
At = pmatrixToRolmip(P.A);
Bt = pmatrixToRolmip(P.B);
Ct = pmatrixToRolmip(P.C);
Dt = pmatrixToRolmip(P.D);

%% Set up LMIs
% TODO (18/05/20): Parameter dependent storage func., requires rate bounds.
% Get structure from pmatrixToRolmip function (second output)
Xt = rolmipvar(nx,nx,'X','sym',0,0);        % Quadratic storage matrix

switch systemType
    % ####    Continious Time     #### %
    case 'ct'
        switch lower(type)
            case 'l2'
                gam = sdpvar(1);
                LMI = [];

                H1 = [At'*Xt+Xt*At, Xt*Bt,        Ct';
                      Bt'*Xt,       -gam*eye(nu), Dt';
                      Ct,           Dt,           -gam*eye(ny)];
                LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nx+nu+ny)];
                LMI = [LMI; Xt>=numAcc*eye(nx)];

                diagnostics = optimize(LMI,gam,solOpt);
                if lpvnormErrorCheck(diagnostics,type)
                    out = NaN;
                    X = NaN;
                else
                    out = value(gam);
                    X = value(Xt);
                end
            case 'h2'
                % Check if D-matrix is zero;
                if checkValidH2Norm(Dt,numAcc)
                    gam = sdpvar(1);
                    LMI = [];

                    H1 = [At'*Xt+Xt*At, Xt*Bt;
                          Bt'*Xt,       -gam*eye(nu)];
                    H2 = [Xt, Ct';
                          Ct, gam*eye(ny)];
                    LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nx+nu)];
                    LMI = [LMI; (H2+H2')*.5>=numAcc*eye(nx+ny)];
                    LMI = [LMI; Xt>=numAcc*eye(nx)];

                    diagnostics = optimize(LMI,gam,solOpt);
                    if lpvnormErrorCheck(diagnostics,type)
                        out = NaN;
                        X = NaN;
                    else
                        out = value(gam);
                        X = value(Xt);
                    end
                else
                    out = Inf;
                    X = NaN;
                end
            case 'l1' % TODO (26/06/2020): Optimize L1 linesearch
                [gam,~,X] = l1Search(Xt,At,Bt,Ct,Dt,...
                    nx,nu,ny,'ct',l1Opt,numAcc,solOpt);
                out = gam;
            case 'passive'
                if checkValidPassivity(nu,ny)
                    LMI = [];
                    H1 = [At'*Xt+Xt*At, Xt*Bt-Ct';
                          Bt'*Xt-Ct,    -Dt'-Dt];
                    LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nx+nu)];
                    LMI = [LMI; Xt>=numAcc*eye(nx)];

                    diagnostics = optimize(LMI,[],solOpt);
                    if lpvnormErrorCheck(diagnostics,type)
                        out = false;
                        X = NaN;
                    else
                        out = true;
                        X = value(Xt);
                    end
                else
                    out = NaN;
                    X = NaN;
                end
        end
    % ####    Discrete Time     #### %
    case 'dt'
        switch lower(type)
            case 'l2'
                gam = sdpvar(1);
                LMI = [];

                H1 = [Xt,           Xt*At,          Xt*Bt,          zeros(nx,ny);
                      At'*Xt,       Xt,             zeros(nx,nu),   Ct';
                      Bt'*Xt,       zeros(nu,nx),   gam*eye(nu), 	Dt';
                      zeros(ny,nx), Ct,             Dt,             gam*eye(ny)];
                LMI = [LMI; (H1+H1')*.5>=numAcc*eye(2*nx+nu+ny)];

                diagnostics = optimize(LMI,gam,solOpt);
                if lpvnormErrorCheck(diagnostics,type)
                    out = NaN;
                    X = NaN;
                else
                    out = value(gam);
                    X = value(Xt);
                end
            case 'h2'
                % Check if D-matrix is zero;
                if checkValidH2Norm(Dt,numAcc)
                    gam = sdpvar(1);
                    LMI = [];

                    H1 = [Xt,       At*Xt,          Bt;
                          Xt*At',   Xt,             zeros(nx,nu);
                          Bt',      zeros(nu,nx),   gam*eye(nu)];
                    H2 = [Xt,           Xt*Ct';
                          Ct*Xt,        gam*eye(ny)];
                    LMI = [LMI; (H1+H1')*.5>=numAcc*eye(2*nx+nu)];
                    LMI = [LMI; (H2+H2')*.5>=numAcc*eye(nx+ny)];

                    diagnostics = optimize(LMI,gam,solOpt);
                    if lpvnormErrorCheck(diagnostics,type)
                        out = NaN;
                        X = NaN;
                    else
                        out = value(gam);
                        X = value(Xt);
                    end
                else
                    out = Inf;
                    X = NaN;
                end
            case 'l1' % TODO (26/06/2020): Optimize L1 linesearch
                [gam,~,X] = l1Search(Xt,At,Bt,Ct,Dt,...
                    nx,nu,ny,'dt',l1Opt,numAcc,solOpt);
                out = gam;
            case 'passive'
                if checkValidPassivity(nu,ny)
                    LMI = [];
                    H1 = [Xt,       At'*Xt,     Ct';
                          Xt*At,    Xt,         Xt*Bt;
                          Ct,       Bt'*Xt,     Dt+Dt'];
                    LMI = [LMI; (H1+H1')*.5>=numAcc*eye(2*nx+nu)];

                    diagnostics = optimize(LMI,[],solOpt);
                    if lpvnormErrorCheck(diagnostics,type)
                        out = false;
                        X = NaN;
                    else
                        out = true;
                        X = value(Xt);
                    end
                else
                    out = NaN;
                    X = NaN;
                end
        end
end
end

%% ########################## LOCAL FUNCTIONS ########################## %%
%% Check if D-matrix is zero for H2-norm
function out = checkValidH2Norm(D,numAcc)
    out = all(abs(cell2mat(coeffs(D)))<=numAcc,'all');
    if ~out
        warning('D-matrix is non-zero, hence the H2 norm is infinite.');
    end
end

%% Check if for passivity number of inputs and outpus are the same
function out = checkValidPassivity(nu,ny)
    out = (nu == ny);
    if ~out
        warning('Passivity check not possible, number of inputs and outputs are not the same.');
    end
end

%% Checks if optimization problem is solved %
function solverError = lpvnormErrorCheck(diagnostics,type)
    if strcmpi(type,'passive')
        errorText = 'Could not compute if the system is passive.\n';
    else
        errorText = sprintf('Could not compute the %s-norm.\n',lower(type));
    end
    errorMessage = [sprintf(errorText),...
        sprintf('Solver returned error code %i: %s. ',diagnostics.problem,...
        diagnostics.info)];
    if diagnostics.problem == 0
        solverError = false;
    else
        warning(errorMessage);
        solverError = true;
    end
end

%% L1 search function %
function [gammaVal, lambda, X] = l1Search(Xt,At,Bt,Ct,Dt,nx,nu,ny,systemType,l1Opt,numAcc,solOpt)
lambdaGrid = l1Opt.lambdaGrid;
accLambda = l1Opt.accuracyLambda;
maxIt = l1Opt.maxIterations;

nLambda = 1; nGamma = 2;
nInit = numel(lambdaGrid);
solLambGamm = zeros(nGamma,nInit);
solLambGamm(nLambda,:) = lambdaGrid;

% Create problem
gam = sdpvar(1);
lambda = sdpvar(1);
mu = rolmipvar('mu','scalar'); % workaround for bug in rolmip
switch systemType
    case 'ct'
        H1 = [At'*Xt+Xt*At+Xt*lambda, (Bt'*Xt)';
              Bt'*Xt,                 -mu*eye(nu)];
        H2 = [Xt*lambda,    zeros(nx,nu),      Ct';
              zeros(nu,nx), -(mu-gam)*eye(nu), Dt';
              Ct,           Dt,                gam*eye(ny)];
        LMI = [(H1+H1')*.5<= -numAcc*eye(nu+nx);
               (H2+H2')*.5>=numAcc*eye(nu+nx+ny);
               Xt>=numAcc*eye(nx);
               mu>=numAcc; gam>=numAcc];
    case 'dt'
        H1 = [Xt*(1-lambda), At'*Xt, zeros(nx,nu);
              Xt*At,         Xt,     (Bt'*Xt)';
              zeros(nu,nx),  Bt'*Xt, mu*eye(nu)];
        H2 = [Xt*lambda,    zeros(nx,nu),      Ct';
              zeros(nu,nx), -(mu-gam)*eye(nu), Dt';
              Ct,           Dt,            gam*eye(ny)];
        LMI = [(H1+H1')*.5<= -numAcc*eye(2*nx+nu);
               (H2+H2')*.5>=numAcc*eye(nu+nx+ny);
               mu>=numAcc; gam>=numAcc];
    otherwise
        error('This should not be possible');
end

% TODO (23/04/2021): Change coefs(Xt) when using param. var. storage
l1Solve = optimizer(LMI,gam,solOpt,lambda,{gam,coefs(Xt)});

% Initial search
[l1Solution,solverProblem] = l1Solve(lambdaGrid);
solLambGamm(nGamma,:) = l1Solution{1};
solLambGamm(nGamma,solverProblem~=0)=NaN; % Set to NaN if not solved correctly

assert(~all(isnan(solLambGamm(nGamma,:))),'No solution found during the initial search')
% Find minimal value of gamma
[gammaval,idx] = min(solLambGamm(nGamma,:));
fprintf('Initial search results:\n\t gamma=%.4g\t\tlambda=%.4g\n',gammaval,solLambGamm(nLambda,idx));
if idx == 1
    lambdamax = solLambGamm(nLambda,idx+1);
elseif idx == nInit
    lambdamin = solLambGamm(nLambda,idx-1);
else
    lambdamax = solLambGamm(nLambda,idx+1);
    lambdamin = solLambGamm(nLambda,idx-1);
end

% Golden ratio search
gr = (sqrt(5) + 1) / 2; % Golden ratio
cnt = 0;                % Reset counter
while ((lambdamax-lambdamin) > accLambda) && cnt < maxIt
    % Create lambda range using golden ratio
    c = lambdamax-(lambdamax-lambdamin)/gr;
    d = lambdamin + (lambdamax-lambdamin)/gr;
    lambdaGrid = [lambdamin, c, d, lambdamax];
    
    % Determine gamma for the golden ratio range
    [l1Solution,solverProblem] = l1Solve(lambdaGrid);
    solGamm = l1Solution{1};
    solGamm(solverProblem~=0)=NaN; % Set to NaN if not solved correctly
    
    if all(isnan(solGamm))
        warning('No solution found during the golden section search')
    end
    
    % determine new lambda range
    [gammaMin,idx] = min(solGamm);
    if     idx == 1; lambdamax = c;
    elseif idx == 2; lambdamax = d;
    elseif idx == 3; lambdamin = c;
    elseif idx == 4; lambdamin = d;
    end
    cnt = cnt+1;
end
gammaVal = gammaMin;
lambda = lambdaGrid(idx);
% TODO (09/04/2021): Add mu as output? Problem with optimizer and rolmipvar
X =  l1Solution{2}(:,(idx-1)*nx+1:idx*nx);
fprintf('Golden section search results:\n\t gamma=%.4g\t\tlambda=%.4g\n',gammaVal,lambda);
end