function [Xt,Gt,At,Bt,Ct,Dt,mapping] = applyNonlinearLmiTransformationDT(genplant,synopts)
%APPLYNONLINEARLMITRANSFORMATIONDT Applies nonlinear transformation to the
%discrete time generalized plant matrices in order to construct LMI. See
%also <a href="matlab:help lpvsyn">lpvsyn</a>.
%
%   Syntax:
%       [Xt,At,Bt,Ct,Dt,mapping] = applyNonlinearLmiTransformationDT(genplant,synopts)
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

%% extract from generalized plant
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
mapping.U = rolmipvar(nx,nx,'U','full',st,rho);
mapping.V = rolmipvar(nx,ny,'V','full',st,rho);
mapping.W = rolmipvar(nu,nx,'W','full',st,rho);
mapping.X = rolmipvar(nu,ny,'X','full',st,rho);

if ~paramVarLyap % Constant
    mapping.Xx = rolmipvar(nx,nx,'Xx','sym',0,0);
    mapping.Xy = rolmipvar(nx,nx,'Xy','full',0,0);
    mapping.Xz = rolmipvar(nx,nx,'Xz','sym',0,0);
else
    error('Parameter-varying Lyapunov implementation not yet implemented for NL transformation method');
end

mapping.J = rolmipvar(nx,nx,'J','full',0,0);
mapping.N = rolmipvar(nx,nx,'N','full',0,0);
mapping.S = rolmipvar(nx,nx,'S','full',0,0);

Xt = [mapping.Xx,  mapping.Xy;
      mapping.Xy', mapping.Xz];
At = [A*mapping.J+Bu*mapping.W, A+Bu*mapping.X*Cy;
      mapping.U,                mapping.N*A+mapping.V*Cy];
Bt = [Bw+Bu*mapping.X*Dyw;
      mapping.N*Bw+mapping.V*Dyw];
Ct = [Cz*mapping.J+Dzu*mapping.W,Cz+Dzu*mapping.X*Cy];
Dt = Dzw+Dzu*mapping.X*Dyw;
Gt = [mapping.J+mapping.J'-mapping.Xx, eye(nx)+mapping.S'-mapping.Xy;
      eye(nx)+mapping.S-mapping.Xy',   mapping.N+mapping.N'-mapping.Xz];
end