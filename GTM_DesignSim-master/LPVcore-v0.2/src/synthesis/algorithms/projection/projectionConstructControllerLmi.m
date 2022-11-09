function [LMI,Theta] = projectionConstructControllerLmi(genplant,X,Xd,Y,gam,numAcc)
%PROJECTIONCONSTRUCTCONTROLLERLMI Constructs linear matrix inequalities for 
%projection based controller synthesis. See also <a href="matlab:help lpvl2syn">lpvl2syn</a>
%and <a href="matlab:help projectionConstructExistenceLmi">projectionConstructExistenceLmi</a>.
%
%   Syntax:
%       [LMI,Thetah] = projectionConstructControllerLmi(genplant,X,Xd,Y,gamma,synopts)
%
%   Inputs:
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help generalizedPlantToRolmip">generalizedPlantToRolmip</a>.
%       X (rolmipvar): X-matrix resulting from the sysnthesis.
%       Xd (rolmipvar): Derivative of the X-matrix resulting from the 
%           sysnthesis. 
%       Y (rolmipvar): Y-matrix resulting from the sysnthesis.
%       gam (double): Upper bound for the L2-gain resulting form the
%           synthesis.
%       synopts (struct): Synthesis options structure. See 
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%       
%   Outputs:
%       LMI: List of linear matrix inequalities as YALMIP constraints that
%           are used for synthesis.
%       Theta: Rolmipvar matrix containing controller variables, i.e.
%           Theta = [Ak, Bk; Ck, Dk].
%

%% Variables
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
nw  = genplant.nw;
nz  = genplant.nz;
ny  = genplant.ny;
nu  = genplant.nu;
rho = genplant.rho;

%% Construct variables
% check if X is rho dependent
if isa(X,'rolmipvar')
    X = value(X);
else
    X = double(X);
end
if isa(Xd,'rolmipvar')
    Xd = value(Xd);
else
    Xd = double(Xd);
end

Y = value(Y);

Ak = rolmipvar(nx,nx,'Ak','full',st,rho);
Bk = rolmipvar(nx,ny,'Bk','full',st,rho);
Ck = rolmipvar(nu,nx,'Ck','full',st,rho);
Dk = rolmipvar(nu,ny,'Dk','full',st,rho);

%% Construct LMIs
Theta = [Ak,Bk;Ck,Dk];
% Closed loop matrices
Acl = [A+Bu*Dk*Cy,Bu*Ck;Bk*Cy,Ak];
Bcl = [Bw+Bu*Dk*Dyw;Bk*Dyw];
Ccl = [Cz+Dzu*Dk*Cy,Dzu*Ck];
Dcl = Dzw+Dzu*Dk*Dyw;

% Set up controller construction LMIs
Yinv = inv(Y);
P = [X,-X+Yinv;-X+Yinv,X-Yinv];
Pd = [Xd,-Xd;-Xd,Xd];

% Continuous time
if genplant.Ts == 0
H1 = [Acl'*P+P*Acl+Pd, P*Bcl,        Ccl';
      Bcl'*P,          -gam*eye(nw), Dcl';
      Ccl,             Dcl,          -gam*eye(nz)];
LMI = (H1+H1')*.5<=-numAcc*eye(2*nx+nw+nz);
% Discrete time
else
H1 = [P,              P*Acl,          P*Bcl,          zeros(2*nx,nz);
      Acl'*P,         P,              zeros(2*nx,nw), Ccl';
      Bcl'*P,         zeros(nw,2*nx), gam*eye(nw),    Dcl';
      zeros(nz,2*nx), Ccl,            Dcl,            gam*eye(nz)];
LMI = (H1+H1')*.5>=numAcc*eye(4*nx+nw+nz);
end
end