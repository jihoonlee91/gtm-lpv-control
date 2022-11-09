function closedLoopMatrices = projectionGetClosedloopMatrices(genplant,mapping)
%PROJECTIONGETCLOSEDLOOPMATRICES Constructs closed-loop plant matrices to
%used during projection method.
%
%   Syntax:
%       closedLoopMatrices = projectionGetClosedloopMatrices(genplant,...
%                                                            mapping)
%
%   Inputs:
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help lpvsynGeneralizedPlantStructure"
%           >lpvsynGeneralizedPlantStructure</a>.
%       mapping (struct): Structure containing the variables that resulted
%           from projection method. See <a href=
%           "matlab:help projectionApply">projectionApply</a>.
%
%   Outputs:
%       closedLoopMatrices: Structure containing all the closed-loop
%           matrices used for controller synthesis.
%
%   See also LPVSYN.
%

%% Extract variables
A   = genplant.A;
Bw  = genplant.Bw;
Bu  = genplant.Bu;
Cz  = genplant.Cz;
Cy  = genplant.Cy;
Dzw = genplant.Dzw;
Dzu = genplant.Dzu;
Dyw = genplant.Dyw;
nx  = genplant.nx;
isCT = genplant.isCT;

Ak = mapping.Ak;
Bk = mapping.Bk;
Ck = mapping.Ck;
Dk = mapping.Dk;

%% Construct closed-loop storage function
if isCT
    % Get vaues
    X  = value(mapping.X);
    Y  = value(mapping.Y);
    Xd = value(mapping.Xd);
    % Yd = value(Yd);   % unused as is zero
    
    % Retransform closed-loop storage:
    % LMI Notes eq. (4.2.15) for construction Xcl with (X(p) and Y)
    % U(p) = (Yinv-X(p)) and V = Y (satisfies I-X(p)Y=U(p)V')
    % resulting in:
    % Xcl(p) = [X(p), inv(Y)-X(p);inv(Y)-X(p),X(p)-Y(inv)];
    % Xcldot = [Xd,-Xd;-Xd,Xd]
    Yinv = inv(Y);
    
    closedLoopMatrices.Xcl    = [X,      Yinv-X; 
                                 Yinv-X, X-Yinv];
    closedLoopMatrices.Xcldot = [Xd,  -Xd;
                                 -Xd, Xd];
else
    % Get values
    S = value(mapping.S);
    N = value(mapping.N);
    J = value(mapping.J);
    
    Xx = value(mapping.Xx);
    Xy = value(mapping.Xy);
    Xz = value(mapping.Xz);
    
    XxPlus = value(mapping.XxPlus);
    XyPlus = value(mapping.XyPlus);
    XzPlus = value(mapping.XzPlus);
    
    % Retransform closed-loop storage:
    % RL = S-NJ
    % Take R = I and L = S-NJ
    closedLoopMatrices.Xcl     = [Xx,        -Xx*N'+Xy;
                                  -N*Xx+Xy', N*Xx*N'-N*Xy-Xy'*N'+Xz];
    closedLoopMatrices.XclPlus = [XxPlus,            -XxPlus*N'+XyPlus;
                                  -N*XxPlus+XyPlus', N*XxPlus*N'-N*XyPlus-XyPlus'*N'+XzPlus];
    closedLoopMatrices.Gcl     = [J,     eye(nx)-J*N';
                                  S-N*J, -(S-N*J)*N'];
end

%% Closed loop matrices
closedLoopMatrices.Acl = [A+Bu*Dk*Cy,Bu*Ck;Bk*Cy,Ak];
closedLoopMatrices.Bcl = [Bw+Bu*Dk*Dyw;Bk*Dyw];
closedLoopMatrices.Ccl = [Cz+Dzu*Dk*Cy,Dzu*Ck];
closedLoopMatrices.Dcl = Dzw+Dzu*Dk*Dyw;
end