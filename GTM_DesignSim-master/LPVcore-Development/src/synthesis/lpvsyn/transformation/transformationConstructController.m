function [K,Xcl] = transformationConstructController(mapping,genplant,synopts)
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

%% Variables
sysType = synopts.systemType;

plant = genplant.Plant;
Ap = plant.A;

A = genplant.A;
Bu = genplant.Bu;
Cy = genplant.Cy;
nx = genplant.nx;
ny = genplant.ny;
nu = genplant.nu;
Ts = genplant.Ts;
isCT = genplant.isCT;
affine = genplant.affine;

%% Construct controller matrices
if isCT
    %% Continuous-time
    X = value(mapping.X);
    Y = value(mapping.Y); assert(isa(Y,'double'),'Y in X_cl is parameter-varying')

    % Bring to one matrix to only call rolmipToPmatrix once
    Omega = value([mapping.K,mapping.L;mapping.M,mapping.N]);

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
    LeftMat = [inv(U),-U\X*Bu;zeros(nu,nx),eye(nu)];    % == inv([U, X*Bu;zeros(nu,nx), eye(nu)])
    RightMat = [inv(V'),zeros(nx,ny);-Cy*Y/V',eye(ny)]; % == inv([V', zeros(nx,ny);Cy*Y, eye(ny)])
    ABCDk = LeftMat*(Omega-XAY)*RightMat;
else
    %% Discrete-time
    Xt = value([mapping.Xx,mapping.Xy;mapping.Xy',mapping.Xz]);
    Gt = value([mapping.J, eye(nx);mapping.S, mapping.N]);
    Omega = value([mapping.U,mapping.V;mapping.W,mapping.X]);
    
    J = Gt(1:nx,1:nx);
    S = Gt(nx+1:end,1:nx);
    N = Gt(nx+1:end,nx+1:end);

    % Compute R & L
    [u,s,v] = svd(S-N*J);
    R = u*sqrt(s);
    L = sqrt(s)*v';

    % Construct closed-loop storage function
    Tinv = [eye(nx),-N'/R';zeros(nx),inv(R')]; % == inv([eye(nx),N';zeros(nx),R'])
    Xcl = Tinv'*Xt*Tinv;
    
    % Construct controller matrices
    NAJ = [N;zeros(nu,nx)]*A*[J,zeros(nx,ny)];
    LeftMat = [inv(R),-R\N*Bu;zeros(nu,nx),eye(nu)]; % == inv([R,N*Bu;zeros(nu,nx),eye(nu)])
    RightMat = [inv(L),zeros(nx,ny);-Cy*J/L,eye(ny)]; % == inv([L,zeros(nx,ny);Cy*J,eye(ny)])
    ABCDk = LeftMat*(Omega-NAJ)*RightMat;
end

% For affine, convert rolmipvar to pmatrix as indexing does not properly
% work for rolmipvars;
switch sysType
    case 'affine'
        st = affine.st;
        ABCDk = rolmipToPmatrix(ABCDk,Ap,st);
        Xcl = rolmipToPmatrix(Xcl,Ap,st);
    case 'grid'
        Xcl = getMatrices(value(Xcl));
end

%% Construct controller
Ak = ABCDk(1:nx,1:nx);
Bk = ABCDk(1:nx,end-ny+1:end);
Ck = ABCDk(end-nu+1:end,1:nx);
Dk = ABCDk(end-nu+1:end,end-ny+1:end);

switch sysType
    case 'affine'
        K = lpvss(Ak,Bk,Ck,Dk,Ts);
    case 'grid'
        samplingGrid = plant.SamplingGrid;

        Ak = getMatrices(Ak);
        Bk = getMatrices(Bk);
        Ck = getMatrices(Ck);
        Dk = getMatrices(Dk);

        sys = ss(Ak,Bk,Ck,Dk,Ts);
        
        K = lpvgridss(sys,samplingGrid);
end
end