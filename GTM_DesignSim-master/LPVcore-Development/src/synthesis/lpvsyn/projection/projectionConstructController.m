function [K,Xcl] = projectionConstructController(genplant,mapping,Xcl,synopts)
%PROJECTIONCONSTRUCTCONTROLLER Constructs the controller resulting from 
%projection based controller synthesis.
%
%   Syntax:
%       K = projectionConstructController(genplant,mapping)
%
%   Inputs:
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help lpvsynGeneralizedPlantStructure"
%           >lpvsynGeneralizedPlantStructure</a>.
%       mapping (struct): Structure containing the variables that resulted
%           from projection method. See <a href=
%           "matlab:help projectionApply">projectionApply</a>.
%       Xcl (rolmipvar): Closed-loop Lyapunov matrix as rolmipvar.
%       synopts (struct): Synthesis options structure. See 
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%       
%   Outputs:
%       K: Constructed controller as lpvss object.
%       Xcl: Closed-loop Lyapunov matrix converted to pmatrix.
%
%   See also LPVSYN.
%

%% Extract variables
nx = genplant.nx;
ny = genplant.ny;
nu = genplant.nu;
plant  = genplant.Plant;
affine = genplant.affine;
Ts = genplant.Ts;

sysType = synopts.systemType;

Theta = mapping.Theta;

%% Extract Controller
switch sysType
    case 'affine'
        st = affine.st;
        stv = affine.stv;

        Theta = rolmipToPmatrix(Theta,plant.A,st);
        Xcl = rolmipToPmatrix(Xcl,plant.A,stv);
    case 'grid'
        Theta = value(Theta);
        Xcl = getMatrices(value(Xcl));
end

Ak = Theta(1:nx,1:nx);
Bk = Theta(1:nx,end-ny+1:end);
Ck = Theta(end-nu+1:end,1:nx);
Dk = Theta(end-nu+1:end,end-ny+1:end);

switch sysType
    case 'affine'
        K = lpvss(Ak,Bk,Ck,Dk,Ts);
    case 'grid'
        % Reconstruct lti array from Ak, Bk, Ck and Dk
        samplingGrid = plant.SamplingGrid;

        Ak = getMatrices(Ak);
        Bk = getMatrices(Bk);
        Ck = getMatrices(Ck);
        Dk = getMatrices(Dk);

        sys = ss(Ak,Bk,Ck,Dk,Ts);
        
        K = lpvgridss(sys,samplingGrid);
end     
end