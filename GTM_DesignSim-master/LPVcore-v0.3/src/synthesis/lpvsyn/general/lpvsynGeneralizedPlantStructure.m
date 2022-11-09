function [genplant,synopts] = lpvsynGeneralizedPlantStructure(P,ny,nu,synopts)
%LPVSYNGENERALIZEDPLANTSTRUCTURE Creates a structure based on the generalized
%plant P that is used for the synthesis algorithms.
%
%   Syntax: 
%       [genplant,synopts] = lpvsynGeneralizedPlantStructure(P,ny,nu,...
%                                                            synopts)
% 
%   Inputs:
%       P (lpvss): LPVSS object that represents the generalized plant.
%       ny (int): The number of measured variables. See <a href=
%           "matlab:help lpvsyn">lpvsyn</a>.
%       nu (int): The number of controlled variables. See <a href=
%           "matlab:help lpvsyn">lpvsyn</a>.
%       synopts (structure): Stucture containing LPV synthesis options, see
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
% 
%   Outputs:
%       genplant: Structure containing the various matrices and dimensions
%           of the generalized plant which are used during synthesis.
%    
%   See also LPVSYN.
%

%% Get variables
parVarStorage = synopts.parameterVaryingStorage;

%% Get constants
% get sampling-time (CT = 0)
Ts = P.Ts;

% determine matrix sizes
nx = P.Nx;
nw = P.Nu-nu;
nz = P.Ny-ny;

% get scheduling variable info
np = P.Np;
assert(np>0,'The system is LTI.');
rho = reshape(cell2mat(P.SchedulingTimeMap.Range),[],2);
v = reshape(cell2mat(P.SchedulingTimeMap.RateBound),[],2);

assert(~any(isinf(rho),'all'),'One or more scheduling-variables have infinite range.');

%% Construct cell array with scheduling-variables
p = cell(np,1);

for ii = 1:np % [A0 + p1 A1 + ... +pnp Anp] 
    At = simplify(P.A); % Use A-matrix to extract the (affine) basis functions
    p{ii} = pmatrix(1,At.bfuncs(ii+1),'SchedulingTimeMap',P.SchedulingTimeMap);
end

%% Construct matrices to be used with rolmip
% Assumes the plant P has the following structure:
%
% [xdot]   [  A |  Bw |  Bu ][x] 
% [  z ] = [ Cz | Dzw | Dzu ][w]
% [  y ]   [ Cy | Dyw |  0  ][u]
%

% Construct rolmip variables
stFull = {};
[A,stFull{1}]   = pmatrixToRolmip(P.A,'A');
[Bw,stFull{2}]  = pmatrixToRolmip(P.B(:,1:nw),'Bw');
[Cz,stFull{3}]  = pmatrixToRolmip(P.C(1:nz,:),'Cz');
[Dzw,stFull{4}] = pmatrixToRolmip(P.D(1:nz,1:nw),'Dzw');

stID = find(~cellfun(@isempty,stFull),1); % find first non-empty element
st = stFull{stID}; 

% Get other matrices
Bu  = simplify(P.B(:,nw+1:end),eps);
Cy  = simplify(P.C(nz+1:end,:),eps);
Dzu = simplify(P.D(1:nz,nw+1:end),eps);
Dyw = simplify(P.D(nz+1:end,1:nw),eps);

% Check if certain matrices are constant
assert(isconst(Bu),'Bu parameter varying');
assert(isconst(Cy),'Cy parameter varying');
assert(isconst(Dzu),'Dzu parameter varying');
assert(isconst(Dyw),'Dyw parameter varying');

Bu = Bu.matrices;
Cy = Cy.matrices;
Dzu = Dzu.matrices;
Dyw = Dyw.matrices;

% Check if feedthrough term is zero
assert(all(simplify(P.D(nz+1:end,nw+1:end),eps) == 0),'Dyu is non-zero');

%% Checks and variables for parameter-varying storage function
% Find rho's which have infinite rate bounds
infV = any(isinf(v),2);

% If all parameter rate bounds are infinite set paramVarStorage to 0
if sum(infV) == np
    if parVarStorage
        warning(['Scheduling-variables have infinite rate bounds. '...
            'Switching to parameter-independent storage function.']);
    end
    parVarStorage = 0;
% Else use param. varying storage (if not forced off by user)
else
    if isempty(parVarStorage)   % parVarStorage = [] by default
        parVarStorage = 1;
    end
end
synopts.parameterVaryingStorage = parVarStorage;

% Create variables when using parameter-varying storage
if parVarStorage
    vBoundedIndex = find(~infV);
    infVIndex = find(infV);

    % Create st variable which only selects ratebounded rho's
    stv = st([1;vBoundedIndex+1]);
    vDummy = v;                 % Dummy variable used for diff. storage
    vDummy(infVIndex,1) = -1;   % function in rolmip, as it is required 
    vDummy(infVIndex,2) = 1;    % to include all parameters.
else
    stv = [];
    vDummy = v;
end

%% Other variables
isCT = (Ts == 0);   % explicit indication if plant is CT (1) or DT (0)

%% Construct genplant structure
genplant.Plant = P;
genplant.A     = A;
genplant.Bw    = Bw;
genplant.Bu    = Bu;
genplant.Cz    = Cz;
genplant.Cy    = Cy;
genplant.Dzw   = Dzw;
genplant.Dzu   = Dzu;
genplant.Dyw   = Dyw;
genplant.st    = st;
genplant.stv   = stv;
genplant.nx    = nx;
genplant.nw    = nw;
genplant.nz    = nz;
genplant.ny    = ny;
genplant.nu    = nu;
genplant.np    = np;
genplant.rho   = rho;
genplant.v     = v;
genplant.vd    = vDummy;
genplant.Ts    = Ts;
genplant.isCT  = isCT;
end