function genplant = generalizedPlantToRolmip(P,ny,nu)
%GENERALIZEDPLANTTOROLMIP Creates a structure based on the generalized
%plant P that is used for the synthesis algorithms.
%
%   Syntax: 
%       genplant = generalizedPlantToRolmip(P,ny,nu)
% 
%   Inputs:
%       P (lpvss): LPVSS object that represents the generalized plant.
%       ny (int): The number of measured variables. See also <a href="matlab:help lpvsyn">lpvsyn</a>.
%       nu (int): The number of controlled variables. See also <a href="matlab:help lpvsyn">lpvsyn</a>.
% 
%   Outputs:
%       genplant: Structure containing the various matrices and dimensions
%           of the generalized plant which are used during synthesis.
%    

disp('Converting system to correct format...');

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
rho = cell2mat(P.SchedulingTimeMap.Range);
v = [-Inf*ones(np,1), Inf*ones(np,1)]; % TODO (2020/09/16): Fix when parameter rate has been added

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

stMat = [stFull{1}{:};stFull{2}{:};stFull{3}{:};stFull{4}{:}];
[~,stId] = unique(stMat,'rows');
st = stFull{stId}; % Get st from all matrices

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

%% Construct genplant structure
genplant.A   = A;
genplant.Bw  = Bw;
genplant.Bu  = Bu;
genplant.Cz  = Cz;
genplant.Cy  = Cy;
genplant.Dzw = Dzw;
genplant.Dzu = Dzu;
genplant.Dyw = Dyw;
genplant.st  = st;
genplant.nx  = nx;
genplant.nw  = nw;
genplant.nz  = nz;
genplant.ny  = ny;
genplant.nu  = nu;
genplant.np  = np;
genplant.p   = p;
genplant.rho = rho;
genplant.v   = v;
genplant.Ts  = Ts;
end