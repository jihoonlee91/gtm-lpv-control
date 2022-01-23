function [Xt,At,Bt,Ct,Dt,mapping] = applyNonlinearLmiTransformationCT(genplant,synopts)
%APPLYNONLINEARLMITRANSFORMATIONCT Applies nonlinear transformation to the
%continuous time generalized plant matrices in order to construct LMI. See
%also <a href="matlab:help lpvsyn">lpvsyn</a>.
%
%   Syntax:
%       [Xt,At,Bt,Ct,Dt,mapping] = applyNonlinearLmiTransformationCT(genplant,synopts)
%
%   Inputs:
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help generalizedPlantToRolmip">generalizedPlantToRolmip</a>.
%       synopts (struct): Synthesis options structure. See 
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%   Outputs:
%       Xt: Xt-matrix for synthesis dependening on new synthesis variables.
%       At: At-matrix for synthesis dependening on new synthesis variables.
%       Bt: Bt-matrix for synthesis dependening on new synthesis variables.
%       Ct: Ct-matrix for synthesis dependening on new synthesis variables.
%       Dt: Dt-matrix for synthesis dependening on new synthesis variables.
%       mapping: Stucture containing the new synthesis variables: X, Y, K, 
%           L, M, N.
%

%% Extract from generalized plant
A   = genplant.A;
Bw  = genplant.Bw;
Bu  = genplant.Bu;
Cz  = genplant.Cz;
Cy  = genplant.Cy;
Dzw = genplant.Dzw;
Dzu = genplant.Dzu;
Dyw = genplant.Dyw;
st  = genplant.st;
nx  = genplant.nx;
ny  = genplant.ny;
nu  = genplant.nu;
rho = genplant.rho;

% Other variables
paramVarLyap = synopts.parameterVaryingStorage;

%% Construct new variables
% create closed loop Lyapunov function
if ~paramVarLyap % Constant
    mapping.X = rolmipvar(nx,nx,'X','sym',0,0);
    mapping.Y = rolmipvar(nx,nx,'Y','sym',0,0);
else
    error('Parameter-varying lyapunov implementation not yet implemented for NL transformation method')
end

mapping.K = rolmipvar(nx,nx,'K','full',st,rho);
mapping.L = rolmipvar(nx,ny,'L','full',st,rho);
mapping.M = rolmipvar(nu,nx,'M','full',st,rho);
mapping.N = rolmipvar(nu,ny,'N','full',st,rho);

Xt = [mapping.Y, eye(nx);
      eye(nx), mapping.X];
At = [A*mapping.Y+Bu*mapping.M, A+Bu*mapping.N*Cy;
      mapping.K,                mapping.X*A+mapping.L*Cy];
Bt = [Bw+Bu*mapping.N*Dyw;
      mapping.X*Bw+mapping.L*Dyw];
Ct = [Cz*mapping.Y+Dzu*mapping.M, Cz+Dzu*mapping.N*Cy];
Dt = Dzw+Dzu*mapping.N*Dyw;
end