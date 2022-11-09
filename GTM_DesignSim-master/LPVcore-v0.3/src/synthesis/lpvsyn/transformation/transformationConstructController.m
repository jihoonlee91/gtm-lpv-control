function [K,Xcl] = transformationConstructController(mapping,genplant)
%transformationConstructControllerCT Performs nonlinear transformation in
%order to construct the continuous time controller.
%
%   Syntax:
%       [K,Xcl] = transformationConstructControllerCT(mapping,genplant,...
%                                                     synopts)
%
%   Inputs:
%       mapping (struct): Structure containing the variables that resulted
%           from the nonlinear transformation. See <a href=
%           "matlab:help transformationApply">transformationApply</a>.
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help lpvsynGeneralizedPlantStructure"
%           >lpvsynGeneralizedPlantStructure</a>.
%
%   Outputs:
%       K: Constructed controller as lpvss object.
%       Xcl: Matrix of closed-loop Lyapunov function, i.e. the Lypunov
%           function is of the form V(x) = x'*Xcl*x. Either constant or
%           parameter dependent.
%
%   See also LPVSYN.
%

%% Initialize parameters
% extract from genplant
A  = genplant.Plant.A;
Bu = genplant.Bu;
Cy = genplant.Cy;
st = genplant.st;
nx = genplant.nx;
ny = genplant.ny;
nu = genplant.nu;
Ts = genplant.Ts;
isCT = genplant.isCT;

%% Construct controller matrices
if isCT
    %% Continuous-time
    % determine value for X and Y
    X = rolmipToPmatrix(mapping.X,A,st);
    Y = rolmipToPmatrix(mapping.Y,A,st); assert(isa(Y,'double'),'Y in X_cl is parameter-varying')
    
    % Bring to one matrix to only call rolmipToPmatrix once
    Omega = rolmipToPmatrix([mapping.K,mapping.L;mapping.M,mapping.N],A,st);
    
    % Construct closed-loop storage function
    % LMI Notes eq. (4.2.15) for construction Xcl with
    % U = (Yinv-X) and V = Y (satisfies I-XY=UV')
    % resulting in:
    % Xcl = [X, inv(Y)-X;inv(Y)-X,X-Y(inv)];
    % Xcldot = [Xd,-Xd;-Xd,Xd]
    Yinv = inv(Y);
    Xcl = [X, Yinv-X; Yinv-X, X-Yinv];
    U = Yinv-X; V = Y;
    
    % Construct controller matrices 
    XAY = [X;zeros(nu,nx)]*A*[Y,zeros(nx,ny)];
    ABCDk = [U, X*Bu;zeros(nu,nx), eye(nu)]\(Omega-XAY)/[V', zeros(nx,ny);Cy*Y, eye(ny)];
else
    %% Discrete-time
    % determine value for X and G
    Xt = rolmipToPmatrix([mapping.Xx,mapping.Xy;mapping.Xy',mapping.Xz],A,st);
    Gt = rolmipToPmatrix([mapping.J, eye(nx);mapping.S, mapping.N],A,st);
    
    J = Gt(1:nx,1:nx);
    S = Gt(nx+1:end,1:nx);
    N = Gt(nx+1:end,nx+1:end);
    
    % Bring to one matrix to only call rolmipToPmatrix once
    Omega = rolmipToPmatrix([mapping.U,mapping.V;mapping.W,mapping.X],A,st);

    % Compute R & L
    [u,s,v] = svd(S-N*J);
    R = u*sqrt(s);
    L = sqrt(s)*v';

    % Construct closed-loop storage function
    T = [eye(nx),N';zeros(nx),R'];
    Xcl = T'\Xt/T;
    
    % Construct controller matrices
    NAJ = [N;zeros(nu,nx)]*A*[J,zeros(nx,ny)];
    ABCDk = [R,N*Bu;zeros(nu,nx),eye(nu)]\(Omega-NAJ)/[L,zeros(nx,ny);Cy*J,eye(ny)];
end

%% Construct controller
Ak = ABCDk(1:nx,1:nx);
Bk = ABCDk(1:nx,end-ny+1:end);
Ck = ABCDk(end-nu+1:end,1:nx);
Dk = ABCDk(end-nu+1:end,end-ny+1:end);

K = lpvss(Ak,Bk,Ck,Dk,Ts);
end