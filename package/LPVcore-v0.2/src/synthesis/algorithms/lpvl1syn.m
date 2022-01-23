function [K,gam,Xcl] = lpvl1syn(P,ny,nu,synopts)
%LPVL1SYN L1-gain suboptimal state-space affine LPV controller synthesis. 
%
%   Syntax:
%       [K, gam, Xcl] = lpvsyn(P, ny, nu, synopts)
%
%   Inputs:
%       P (lpvss): LPV state-space system with affine parameter depenency
%           for which the controller is synthesized.
%       ny (int): The number of measured variables, i.e. the last ny 
%           outputs of the plant P used as inputs to the controller K.
%       nu (int): The number of controlled variables, i.e. the last nu
%           inputs of the plant P that are connected to the output of the
%           controller K.
%       synopts (structure): Stucture containing LPV synthesis options, see
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%	Outputs:
%       K: Suboptimal state-space affine controller as lpvss object.
%       gam: Upperbound for the gain of the closed-loop system.
%       Xcl: Matrix of closed-loop storage/Lyapunov function, i.e. the
%           Lypunov function is of the form V(x) = x'*Xcl*x. Either 
%           constant or parameter dependent.
%

%% Convert LPVCore generalized plant to rolmip variables
genplant = generalizedPlantToRolmip(P,ny,nu);

%% Variables
nx = genplant.nx;
nw = genplant.nw;
nz = genplant.nz;

backoff = synopts.backoff;
numAcc = synopts.numericalAccuracy;
solOpt = synopts.solverOptions;

%% Synthesis
%% Continuous time
if genplant.Ts == 0
    % L1 controller synthesis is for now based on a NL transformation
    disp('Applying NL transformation...')
    [Xt,At,Bt,Ct,Dt,mapping] = applyNonlinearLmiTransformationCT(genplant,synopts);
    
    % Solve and search over lambda
    disp('Solving transformed synthesis LMI...')
    [gam, lambda, mu] = l1Search(Xt,At,Bt,Ct,Dt,nx,nw,nz,synopts);
    
    % Solve with Backoff
    LMI = [];
    gam = gam*backoff;
    H1 = [At'+At+lambda*Xt, Bt;
          Bt',              -mu*eye(nw)];
    H2 = [lambda*Xt,      zeros(2*nx,nw),    Ct';
          zeros(nw,2*nx), -(mu-gam)*eye(nw), Dt';
          Ct,             Dt,                gam*eye(nz)];
    LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nw+2*nx)];
    LMI = [LMI; (H2+H2')*.5>=numAcc*eye(nw+2*nx+nz)];
    
    % Solve synthesis LMI's with backoff
    disp('Solving backoff synthesis LMI...')
    diagnostics = optimize(LMI,[],solOpt);
    
    assert(diagnostics.problem == 0,'Controller backoff synthesis failed');
    
    % Construct controller
    disp('Constructing controller...')
    [K,Xcl] = nonlinTransformConstructControllerCT(mapping,genplant,synopts);
%% Discrete time [tbd]
else
    error('DT not implemented yet');
end
end

%% Local functions
%% L1 search function %
function [gammaVal, lambda, mu] = l1Search(Xt,At,Bt,Ct,Dt,nx,nw,nz,synopts)
numAcc = synopts.numericalAccuracy;
solOpt = synopts.solverOptions;
l1Opt = synopts.l1Options;

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

H1 = [At'+At+Xt*lambda, Bt;
      Bt',              -mu*eye(nw)];
H2 = [Xt*lambda,      zeros(2*nx,nw),    Ct';
      zeros(nw,2*nx), -(mu-gam)*eye(nw), Dt';
      Ct,             Dt,                gam*eye(nz)];
LMI = [(H1+H1')*.5<=-numAcc*eye(nw+2*nx);
       (H2+H2')*.5>=numAcc*eye(nw+2*nx+nz);
       (Xt+Xt')*.5>=numAcc*eye(2*nx);
       mu>=numAcc; gam>=numAcc];
   
l1Solve = optimizer(LMI,gam,solOpt,lambda,{gam,coefs(mu)});

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
mu =  l1Solution{2}(idx);
fprintf('Golden section search results:\n\t gamma=%.4g\t\tlambda=%.4g\n',gammaVal,lambda);
end