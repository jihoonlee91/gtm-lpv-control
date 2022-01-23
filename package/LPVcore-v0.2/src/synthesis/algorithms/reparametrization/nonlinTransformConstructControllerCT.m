function [K,Xcl] = nonlinTransformConstructControllerCT(mapping,genplant,synopts)
%NONLINTRANSFORMCONSTRUCTCONTROLLERCT Performs nonlinear transformation in
%order to construct the continuous time controller. See also <a href="matlab:help lpvsyn">lpvsyn</a>.
%
%   Syntax:
%       [K,Theta,Xcl] = nonlinTransformConstructControllerCT(mapping,genplant,synopts)
%
%   Inputs:
%       mapping (struct): Structure containing the variables that resulted
%           from the nonlinear transformation. See 
%           <a href="matlab:help applyNonlinearLmiTransformationCT">applyNonlinearLmiTransformationCT</a>.
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

% Other variables
paramVarLyap = synopts.parameterVaryingStorage;

% determine value for X and Y
X = value(mapping.X);
Y = value(mapping.Y); assert(isa(Y,'double'),'Y in X_cl is parameter-varying')

% Bring to one matrix to avoid errors with evalpar
omega = [value(mapping.K),value(mapping.L);
         value(mapping.M),value(mapping.N)];
omega0 = evalpar(omega,st{1});
A0 = evalpar(A,st{1});

Ak = cell(1,np+1);
Bk = cell(1,np+1);
Ck = cell(1,np+1);
Dk = cell(1,np+1);

% construct matrices U and V choose Y for the inversion, since Y is never
% dependent on the scheduling variable [1]
V = Y;
if ~paramVarLyap % constant Xcl
    assert(isa(X,'double'),'X in X_cl is parameter-varying')
    U = Y\eye(nx)-X;
    Xcl = [Y, V; eye(nx), zeros(nx)]\[eye(nx), zeros(nx); X, U];
    Xi = X;
else
    Xcl = cell(1,np+1);
    X0 = evalpar(X,st{1});
end

%% construct controller matrices (and closed-loop lyapunov matrix)
for ii = 1:np+1
    % determine the value of KLMN and A for rho_i
    if ii == 1
        omega_i = omega0;
        Ai = A0;
        if paramVarLyap % parameter varying Xcl
         Xi = X0;
         U = Y\eye(nx)-Xi;
         Xcl{ii} = [Y, V; eye(nx), zeros(nx)]\[eye(nx), zeros(nx); Xi, U];
        end
    else
        omega_i = evalpar(omega,st{ii})-omega0;
        Ai      = evalpar(A,st{ii})-A0;
        if paramVarLyap % parameter varying Xcl
         Xi = evalpar(X,st{ii})-X0;
         U = Y\eye(nx)-Xi;
         Xcl{ii} = [Y, V; eye(nx), zeros(nx)]\[eye(nx), zeros(nx); Xi, U];
        end
    end
    Ki = omega_i(1:nx,1:nx);
    Li = omega_i(1:nx,end-ny+1:end);
    Mi = omega_i(end-nu+1:end,1:nx);
    Ni = omega_i(end-nu+1:end,end-ny+1:end);
    % calculate the matrix [Ak,Bk;Ck,Dk]
    ABCDi = [U, Xi*Bu;zeros(nu,nx), eye(nu)]\[Ki-Xi*Ai*Y, Li; Mi, Ni]/[V', zeros(nx,ny);Cy*Y, eye(ny)];
    % Construct controller matrices
    Ak{ii} = ABCDi(1:nx,1:nx);
    Bk{ii} = ABCDi(1:nx,end-ny+1:end);
    Ck{ii} = ABCDi(end-nu+1:end,1:nx);
    Dk{ii} = ABCDi(end-nu+1:end,end-ny+1:end);
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

K = lpvss(ak,bk,ck,dk);
end