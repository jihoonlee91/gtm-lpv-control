function [out, X] = lpvnorm(P, type, varargin)
%LPVNORM computes the Linf, L2 or H2 norm of a lpvss object or allows to
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
%       type (char): Type of norm to be computed, supported norms: 'linf',
%           'l2', 'h2' and 'passive'. If type is not given, 'l2' is used.
%       
%       Optional name-value pair:
%           'SolverOptions': Structure containing the YALMIP solver 
%               settings. See <a href="matlab:help sdpsettings
%               ">sdpsettings</a>. 
%               Default value: sdpsettings('verbose',0,'solver','sdpt3');
%           'NumericalAccuracy': Numerical accuracy parameter. Used for, 
%               among other things, solving the LMIs (LMI <= NumAcc).
%               Default value: 1e-10;
%           'LinfOptions': Structure containing the options for Linf 
%               analysis. See <a href="matlab:help linfOptions
%               ">linfOptions</a>.
%           'ParameterVaryingStorage': Option to indicate whether a
%               parameter-varying storage function is considered (if at
%               least one of the scheduling-variables has finite
%               rate-bounds). Use 0 to indicate to use a parameter
%               independent storage function, 1 to indicate parameter
%               dependency on all parameters with finite rate-bounds.
%               Default value: 1;
%           'Verbose': Turns progress messages on (1) or off (0);
%               Default value: 1;
%               
%   Outputs:
%       out: Value of the norm in case of the 'linf', 'l2' or 'h2' norm or
%           true in the case of a passivity check and the system is 
%           passive.
%       X: Value of quadratic storage function matrix.      
%

%% Checks
% Check if type of norm is given
if ~exist('type','var')
    warning('No norm type given, computing L2 norm');
    type = 'l2';
% Check if norm type is supported
else
    normList = {'linf','l2','h2','passive'};
    assert(any(strcmpi(type,normList)),...
        sprintf('Unknown system norm.\nSupported system norms: linf, l2, h2, passive.'));
end

% Check type
if isa(P,'lpvgridss')
    sysType = 'grid';
elseif isa(P,'lpvss')
    assert(all([isaffine(P.A),isaffine(P.B),isaffine(P.C),isaffine(P.D)]),...
        'System is not affinely dependent on the parameter');
    sysType = 'affine';
else
    error('Unsupported system type.')
end

% Performance specific checks
switch type
    case 'h2'
        if ~checkValidH2Norm(P.D,1e-10)
            out = Inf;
            X = NaN;
            return;
        end
    case 'passive'
        if ~checkValidPassivity(P.Nu,P.Ny)
            out = NaN;
            X = NaN;
            return;
        end
end

%% Parse optional arguments
% Default values
solOpt = sdpsettings('verbose',0,'solver','sdpt3');
numAcc = 1e-10;
linfOpt = linfOptions;
paramVarStorage = 1;
verbose = 1;

% Parse
opt = inputParser;

checkStruct = @(x) isa(x,'struct');
checkParVarInput = @(x) x == 0 || x == 1;

addParameter(opt,'SolverOptions',solOpt,checkStruct);
addParameter(opt,'NumericalAccuracy',numAcc,@isnumeric);
addParameter(opt,'LinfOptions',linfOpt,@(x)isa(x,'linfOptions'));
addParameter(opt,'ParameterVaryingStorage',paramVarStorage,checkParVarInput);
addParameter(opt,'Verbose',verbose,@isnumeric);

parse(opt,varargin{:});

opt = opt.Results;
paramVarStorage = opt.ParameterVaryingStorage;

opt.SolverOptions.cachesolvers = 1;    % enable caching of solvers for small speed-up

opt.type = lower(type);     % add performance type to opt structure
opt.sysType = sysType;

%% Variables
% check if system is CT or DT
isCT = (P.Ts == 0);
opt.isCT = isCT;

%% Construct variables
switch sysType
    case 'affine'
        [At,stFull{1}] = pmatrixToRolmip(P.A);
        [Bt,stFull{2}] = pmatrixToRolmip(P.B);
        [Ct,stFull{3}] = pmatrixToRolmip(P.C);
        [Dt,stFull{4}] = pmatrixToRolmip(P.D);
        
        % Get st from all matrices
        stID = find(~cellfun(@isempty,stFull),1); % find first non-empty element
        if isempty(stID)
            st = 0;
        else
            st = stFull{stID};
        end
        
        matrices.At = At;
        matrices.Bt = Bt;
        matrices.Ct = Ct;
        matrices.Dt = Dt;
    case 'grid'
        matrices.At = gridvar(P.A);
        matrices.Bt = gridvar(P.B);
        matrices.Ct = gridvar(P.C);
        matrices.Dt = gridvar(P.D);
end

%% Checks and variables for parameter-varying storage function
switch sysType
    case 'affine'
        rhoRange = reshape(cell2mat(P.SchedulingTimeMap.Range),[],2);
        rhoRateBound = reshape(cell2mat(P.SchedulingTimeMap.RateBound),[],2);
        
        % Find rho's which have finite rate bounds
        infRateBound = any(isinf(rhoRateBound),2);
        
        % If all parameter rate bounds are infinite set paramVarStorage to 0
        if sum(infRateBound) == P.Np
            paramVarStorage = 0;
            opt.ParameterVaryingStorage = 0;
        end
        
        % Create variables when using parameter-varying storage
        if paramVarStorage
            rateBoundedRhoIndex = find(~infRateBound);
            infRateBoundIndex = find(infRateBound);
        
            % Create st variable which only selects ratebounded rho's
            stDep = st([1;rateBoundedRhoIndex+1]);
            rhoRateBoundDummy = rhoRateBound;               % Dummy variable used for diff. storage
            rhoRateBoundDummy(infRateBoundIndex,1) = -1;    % function in rolmip, as it is required 
            rhoRateBoundDummy(infRateBoundIndex,2) = 1;     % to include all parameters.
        else
            stDep = [];
            rhoRateBoundDummy = [];
            rateBoundedRhoIndex = [];
        end
        
        stStruct.stDep = stDep;
        stStruct.rhoRange = rhoRange;
        stStruct.rhoRateBoundDummy = rhoRateBoundDummy;
        stStruct.rateBoundedRhoIndex = rateBoundedRhoIndex;
    case 'grid'
        % To be implemented
        opt.ParameterVaryingStorage = 0;
        stStruct = [];
end

%% Solve LMIs
[gam,LMI,Xt,var] = lpvnormConstructAnalysisLmi(matrices,P,stStruct,opt);
[out,X] = lpvnormSolveLmi(LMI,gam,Xt,var,stStruct,P,opt);
end

%% ########################## LOCAL FUNCTIONS ########################## %%
%% Check if D-matrix is zero for H2-norm
function out = checkValidH2Norm(D,numAcc)
    if isa(D,'pmatrix')
        out = all(abs(freeze(D,exp(1)))<=numAcc,'all');
    else
        out = all(abs(D)<=numAcc,'all');
    end
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