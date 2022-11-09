function [K,Xcl] = nonlinTransformConstructControllerDT(mapping,genplant,synopts)
%NONLINTRANSFORMCONSTRUCTCONTROLLERDT Performs nonlinear transformation in
%order to construct the discrete time controller. See also <a href="matlab:help lpvsyn">lpvsyn</a>.
%
%   Syntax:
%       [K,Theta,Xcl] = nonlinTransformConstructControllerDT(mapping,genplant,synopts)
%
%   Inputs:
%       mapping (struct): Structure containing the variables that resulted
%           from the nonlinear transformation. See 
%           <a href="matlab:help applyNonlinearLmiTransformationDT">applyNonlinearLmiTransformationDT</a>.
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help generalizedPlantToRolmip">generalizedPlantToRolmip</a>.
%       synopts (struct): Synthesis options structure. See 
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%   Outputs:
%       K: Constructed controller as lpvss object.
%       Theta: Cell array containing the affine controller matrices where:
%           Theta = { Ak, Bk; Ck, Dk }
%           where Ak is the affine parameter dependent matrix of the form
%           Ak = Ak0 + Ak1 * rho_1 + Ak2 *rho_2 + ...
%           where Ak{1} = Ak0, Ak{2} = Ak1, Ak{3} = Ak2, ... .
%       Xcl: Matrix of closed-loop Lyapunov function, i.e. the Lypunov
%           function is of the form V(x) = x'*Xcl*x. Either constant or
%           parameter dependent.
%

%% Initialize parameters
% extract from genplant
A  = genplant.A;
Bu = genplant.Bu;
Cy = genplant.Cy;
st = genplant.st;
nx = genplant.nx;
ny = genplant.ny;
nu = genplant.nu;
np = genplant.np;
p  = genplant.p;
Ts = genplant.Ts;

% Other variables
paramVarLyap = synopts.parameterVaryingStorage;

% Determine mapping values
U = value(mapping.U);
V = value(mapping.V);
W = value(mapping.W);
X = value(mapping.X);

N = value(mapping.N);
J = value(mapping.J);
S = value(mapping.S);

Xx = value(mapping.Xx);
Xy = value(mapping.Xy);
Xz = value(mapping.Xz);

Xt = [Xx,Xy;Xy',Xz];

if paramVarLyap
    warning('Parameter varying Lyapunov matrix is not (yet) supported, using constant matrix');
end

%% Compute controller and closed-loop matrices
UVWX = [U,V;W,X];

[u,s,v] = svd(S-N*J);
R = u*sqrt(s);
L = sqrt(s)*v';

LeftM = [inv(R),-R\N*Bu;zeros(nu,nx),eye(nu)];
RightM = [inv(L),zeros(nx,ny);-Cy*J/L,eye(ny)];

Ti = [eye(nx),-N'/R';zeros(nx),inv(R')];

Xcl = Ti'*Xt*Ti;

NAJ = [N;zeros(nu,nx)]*A*[J,zeros(nx,ny)];

ThetaMat = LeftM*(UVWX-NAJ)*RightM;

%% Construct controller
Ak = cell(1,np+1);
Bk = cell(1,np+1);
Ck = cell(1,np+1);
Dk = cell(1,np+1);
Theta_ = cell(1,np+1);

Theta_{1} = evalpar(ThetaMat,st{1});

for i = 1:np+1
    if i > 1
        Theta_{i} = evalpar(ThetaMat,st{i})-Theta_{1};
    end
    Ak{i} = Theta_{i}(1:nx,1:nx);
    Bk{i} = Theta_{i}(1:nx,end-ny+1:end);
    Ck{i} = Theta_{i}(end-nu+1:end,1:nx);
    Dk{i} = Theta_{i}(end-nu+1:end,end-ny+1:end);
end

% construct lpvss system
ak = Ak{1};
bk = Bk{1};
ck = Ck{1};
dk = Dk{1};

for i = 2:np+1
    ak = ak+Ak{i}*p{i-1};
    bk = bk+Bk{i}*p{i-1};
    ck = ck+Ck{i}*p{i-1};
    dk = dk+Dk{i}*p{i-1};
end

K = lpvss(ak,bk,ck,dk,Ts);

end