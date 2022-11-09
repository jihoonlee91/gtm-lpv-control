function [genplant,synopts] = lpvsynGeneralizedPlantStructure(P,ny,nu,synopts)
%LPVSYNGENERALIZEDPLANTSTRUCTURE Creates a structure based on the 
%generalized plant P that is used for the synthesis algorithms.
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
sysType = synopts.SystemType;

%% Get constants
% get sampling-time (CT = 0)
Ts = P.Ts;

% determine matrix sizes
nx = P.Nx;
nw = P.Nu-nu;
nz = P.Ny-ny;

% get scheduling variable info
np = P.Np;

affine = struct;
grid = struct;

switch sysType
    case 'affine'
        affine.rho = reshape(cell2mat(P.SchedulingTimeMap.Range),[],2);
        affine.v = reshape(cell2mat(P.SchedulingTimeMap.RateBound),[],2);
        
        assert(~any(isinf(affine.rho),'all'),'One or more scheduling-variables have infinite range.');
end

isCT = (Ts == 0);   % explicit indication if plant is CT (1) or DT (0)

%% Construct matrices to be used with rolmip
% Assumes the plant P has the following structure:
%
% [xdot]   [  A |  Bw |  Bu ][x] 
% [  z ] = [ Cz | Dzw | Dzu ][w]
% [  y ]   [ Cy | Dyw | Dyu ][u]
%
% where Dyu = 0
%

A = P.A;    B = P.B;
C = P.C;    D = P.D;

switch sysType
    case 'affine'
        Bw = B(:,1:nw);         Bu = B(:,nw+1:end);
        Cz = C(1:nz,:);         Cy = C(nz+1:end,:);   
        Dzw = D(1:nz,1:nw);     Dzu = D(1:nz,nw+1:end);
        Dyw = D(nz+1:end,1:nw); Dyu = D(nz+1:end,nw+1:end);

        % Check if feedthrough term (Dyu) is zero
        assert(all(simplify(Dyu,1e-10) == 0,'all'),'Dyu is non-zero');

        % Construct rolmip variables
        stFull = {};
        [A,stFull{1}]   = pmatrixToRolmip(A,'A');
        [Bw,stFull{2}]  = pmatrixToRolmip(Bw,'Bw');
        [Cz,stFull{3}]  = pmatrixToRolmip(Cz,'Cz');
        [Dzw,stFull{4}] = pmatrixToRolmip(Dzw,'Dzw');
        
        stID = find(~cellfun(@isempty,stFull),1); % find first non-empty element
        if isempty(stID)
            affine.st = 0;
            affine.rho = 0;
        else
            affine.st = stFull{stID};
        end
        
        % Other matrices
        Bu  = simplify(Bu,eps);
        Cy  = simplify(Cy,eps);
        Dzu = simplify(Dzu,eps);
        Dyw = simplify(Dyw,eps);
        
        % Check if certain matrices are constant
        assert(isconst(Bu),'Bu parameter varying');
        assert(isconst(Cy),'Cy parameter varying');
        assert(isconst(Dzu),'Dzu parameter varying');
        assert(isconst(Dyw),'Dyw parameter varying');
        
        Bu = Bu.matrices;
        Cy = Cy.matrices;
        Dzu = Dzu.matrices;
        Dyw = Dyw.matrices;
    case 'grid'
        A = gridvar(A);     B = gridvar(B);
        C = gridvar(C);     D = gridvar(D);

        Bw = B(:,1:nw);         Bu = B(:,nw+1:end);
        Cz = C(1:nz,:);         Cy = C(nz+1:end,:);   
        Dzw = D(1:nz,1:nw);     Dzu = D(1:nz,nw+1:end);
        Dyw = D(nz+1:end,1:nw); Dyu = D(nz+1:end,nw+1:end);

        % Check if feedthrough term (Dyu) is zero
        assert(all(abs(getMatrices(Dyu)) <= 1e-10, 'all'), 'Dyu is non-zero');
end


%% Checks and variables for parameter-varying storage function
switch sysType
    case 'affine'
        % Find rho's which have infinite rate bounds
        infV = any(isinf(affine.v),2);
        
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
            affine.stv = affine.st([1;vBoundedIndex+1]);
            affine.vd = affine.v;          % Dummy variable used for diff. storage
            affine.vd(infVIndex,1) = -1;   % function in rolmip, as it is required 
            affine.vd(infVIndex,2) = 1;    % to include all parameters.

            if isCT
                % Force projection for CT with varying storage (for now,
                % 28/03/2022, only projection supports varying storage for
                % CT systems)
                synopts.method = 'projection';
            end
        else
            affine.stv = [];
            affine.vd = affine.v;
        end
    case 'grid'
        % Todo (16/02/22): Grid-based parameter varying storage
        grid.ng = A.Ng;
        grid.dim = A.dim;

        parVarStorage = 0;
        synopts.parameterVaryingStorage = parVarStorage;
end

%% Construct genplant structure
genplant.Plant  = P;
genplant.A      = A;
genplant.Bw     = Bw;
genplant.Bu     = Bu;
genplant.Cz     = Cz;
genplant.Cy     = Cy;
genplant.Dzw    = Dzw;
genplant.Dzu    = Dzu;
genplant.Dyw    = Dyw;
genplant.nx     = nx;
genplant.nw     = nw;
genplant.nz     = nz;
genplant.ny     = ny;
genplant.nu     = nu;
genplant.np     = np;
genplant.affine = affine;
genplant.grid   = grid;
genplant.Ts     = Ts;
genplant.isCT   = isCT;
end