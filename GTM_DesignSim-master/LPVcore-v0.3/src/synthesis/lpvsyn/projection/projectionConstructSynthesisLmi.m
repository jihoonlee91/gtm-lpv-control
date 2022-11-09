function [gam,LMI,var] = projectionConstructSynthesisLmi(matrices,mapping,genplant,synopts)
%PROJECTIONCONSTRUCTSYNTHESISLMI Constructs the synthesis LMIs that are
%used to solve the projected controller synthesis problem.
%
%   Syntax:
%       [gam,LMI,var] = projectionConstructSynthesisLmi(matrices,...
%                           mapping,genplant,synopts)
%
%   Inputs:
%       matrices (struct): Structure containing closed-loop projected 
%           matrices. See <a href="matlab:help projectionApply"
%           >projectionApply</a>.
%       mapping (struct): Structure containing the variables that resulted
%           from projection method. See <a href=
%           "matlab:help projectionApply">projectionApply</a>.
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help lpvsynGeneralizedPlantStructure"
%           >lpvsynGeneralizedPlantStructure</a>.
%       synopts (struct): Synthesis options structure. See <a href=
%           "matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%   Outputs:
%       gam: Closed-loop gain objective variable as sdpvar, empty in case 
%           passivity.
%       LMI: LMI array containing the LMI constraints for the transformed
%           synthesis problem.
%       var: Structure containing other optimization variables. (Only used
%           for L1 based synthesis).
%
%   See also LPVSYN.
%


%% Variables
nx = genplant.nx;
nw = genplant.nw;
nz = genplant.nz;
isCT = genplant.isCT;

perfType = synopts.performance;
numAcc = synopts.numericalAccuracy;

%% LMIs
LMI = [];
var = [];

% Extract matrix variables
Xr = matrices.Xr;
Ar = matrices.Ar;
Br = matrices.Br;
Cr = matrices.Cr;
Dr = matrices.Dr;
N = matrices.N;

if isCT
    %% Continuous-time
    switch perfType
        case 'l2'
            gam = sdpvar(1);
    
            nNx = size(N.X,2);
            nNy = size(N.Y,2);
    
            H1 = N.X'*[Ar.X+Ar.X', Br.X,         Cr.X';
                       Br.X',      -gam*eye(nw), Dr';
                       Cr.X,       Dr,           -gam*eye(nz)]*N.X;
            H2 = N.Y'*[Ar.Y+Ar.Y', Cr.Y',        Br.Y;
                       Cr.Y,       -gam*eye(nz), Dr;
                       Br.Y',      Dr',          -gam*eye(nw)]*N.Y;
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nNx)];
            LMI = [LMI; (H2+H2')*.5<=-numAcc*eye(nNy)];
            LMI = [LMI; (Xr+Xr')*.5>=numAcc*eye(2*nx)];
        case 'passive'
            gam = [];
    
            nNx = size(N.X,2);
            nNy = size(N.Y,2);
    
            H1 = N.X'*[Ar.X+Ar.X', Br.X-Cr.X';
                       Br.X'-Cr.X, -Dr-Dr']*N.X;
            H2 = N.Y'*[Ar.Y+Ar.Y', Br.Y-Cr.Y';
                       Br.Y'-Cr.Y, -Dr-Dr']*N.Y;
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nNx)];
            LMI = [LMI; (H2+H2')*.5<=-numAcc*eye(nNy)];
            LMI = [LMI; (Xr+Xr')*.5>=numAcc*eye(2*nx)];
        case 'h2'
            gam = sdpvar(1);
    
            nNx1 = size(N.X1,2);
            nNy1 = size(N.Y1,2);
            nNx2 = size(N.X2,2);
            nNy2 = size(N.Y2,2);
            
            H1 = N.X1'*[Ar.X+Ar.X', Br.X;
                        Br.X',      -gam*eye(nw)]*N.X1;
            H2 = N.Y1'*[Ar.Y+Ar.Y', Br.Y;
                        Br.Y',      -gam*eye(nw)]*N.Y1;
            H3 = N.X2'*[mapping.X, Cr.X';
                        Cr.X,      gam*eye(nz)]*N.X2;
            H4 = N.Y2'*[Xr,          [Cr.Y';Cr.X'];
                        [Cr.Y,Cr.X], gam*eye(nz)]*N.Y2;
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nNx1)];
            LMI = [LMI; (H2+H2')*.5<=-numAcc*eye(nNy1)];
            LMI = [LMI; (H3+H3')*.5>=numAcc*eye(nNx2)];
            LMI = [LMI; (H4+H4')*.5>=numAcc*eye(nNy2)];
        case 'l1'
            gam = sdpvar(1);
            lambda = sdpvar(1);
            mu = rolmipvar('mu','scalar'); % workaround for bug in rolmip

            nNx1 = size(N.X1,2);
            nNy1 = size(N.Y1,2);
            nNx2 = size(N.X2,2);
            nNy2 = size(N.Y2,2);
            
            H1 = N.X1'*[Ar.X+Ar.X'+mapping.X*lambda, Br.X;
                        Br.X',                       -mu*eye(nw)]*N.X1;
            H2 = N.Y1'*[Ar.Y+Ar.Y'+mapping.Y*lambda, Br.Y;
                        Br.Y',                       -mu*eye(nw)]*N.Y1;
            H3 = N.X2'*[mapping.X*lambda, zeros(nx,nw),      Cr.X';
                        zeros(nw,nx),     -(mu-gam)*eye(nw), Dr';
                        Cr.X,             Dr,                gam*eye(nz)]*N.X2;
            H4 = N.Y2'*[Xr*lambda,      zeros(2*nx,nw),    [Cr.Y';Cr.X'];
                        zeros(nw,2*nx), -(mu-gam)*eye(nw), Dr';
                        [Cr.Y,Cr.X],    Dr,                gam*eye(nz)]*N.Y2;
            LMI = [LMI; (H1+H1')*.5 <= -numAcc*eye(nNx1)];
            LMI = [LMI; (H2+H2')*.5 <= -numAcc*eye(nNy1)];
            LMI = [LMI; (H3+H3')*.5 >= numAcc*eye(nNx2)];
            LMI = [LMI; (H4+H4')*.5 >= numAcc*eye(nNy2)];

            var.mu = mu;
            var.lambda = lambda;
    end
else
    %% Discrete-time
    % Extract extra matrices for DT
    XrPlus = matrices.XrPlus;
    Gr = matrices.Gr;

    switch perfType
        case 'l2'
            gam = sdpvar(1);
    
            nNu = size(N.U,2);
            nNy = size(N.Y,2);
            
            H1 = N.U'*[XrPlus.U,  zeros(nx,nz), Ar.U,            Br.U;
                       rz(nz,nx), gam*eye(nz),  Cr.U,            Dr;
                       Ar.U',     Cr.U',        Gr.U+Gr.U'-Xr.U, zeros(2*nx,nw);
                       Br.U',     Dr',          zeros(nw,2*nx),  gam*eye(nw)]*N.U;
            H2 = N.Y'*[XrPlus.Y,    zeros(2*nx,nz), Ar.Y,            Br.Y;
                       rz(nz,2*nx), gam*eye(nz),    Cr.Y,            Dr;
                       Ar.Y',       Cr.Y',          Gr.Y+Gr.Y'-Xr.Y, zeros(nx,nw);
                       Br.Y',       Dr',            zeros(nw,nx),    gam*eye(nw)]*N.Y;
            LMI = [LMI; (H1+H1')*.5>=numAcc*eye(nNu)];
            LMI = [LMI; (H2+H2')*.5>=numAcc*eye(nNy)];
        case 'passive'
            error(['Not (yet) available, use transformation method. ',...
                'See help lpvsynOptions.']);
        case 'h2'
            error(['Not (yet) available, use transformation method. ',...
                'See help lpvsynOptions.']);
        case 'l1'
            error(['Not (yet) available, use transformation method. ',...
                'See help lpvsynOptions.']);
    end
end
end

%% Create zero matrix as rolmipvariable
function out = rz(n,m)
    out = rolmipvar(1,1,'z',0,0)*zeros(n,m);
end