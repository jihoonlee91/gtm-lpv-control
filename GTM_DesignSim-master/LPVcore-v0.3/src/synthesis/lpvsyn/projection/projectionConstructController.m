function [K,Xcl] = projectionConstructController(genplant,mapping,Xcl)
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
%       
%   Outputs:
%       K: Constructed controller as lpvss object.
%       Xcl: Closed-loop Lyapunov matrix converted to pmatrix.
%
%   See also LPVSYN.
%

%% Extract variables
st = genplant.st;
stv = genplant.stv;
nx = genplant.nx;
ny = genplant.ny;
nu = genplant.nu;
P  = genplant.Plant;

ThetaRolmip = mapping.Theta;

%% Convert to pmatrix
Theta = rolmipToPmatrix(ThetaRolmip,P.A,st);
Xcl = rolmipToPmatrix(Xcl,P.A,stv);

%% Extract Controller
Ak = Theta(1:nx,1:nx);
Bk = Theta(1:nx,end-ny+1:end);
Ck = Theta(end-nu+1:end,1:nx);
Dk = Theta(end-nu+1:end,end-ny+1:end);

K = lpvss(Ak,Bk,Ck,Dk);
end