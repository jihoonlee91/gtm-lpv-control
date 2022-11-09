function [gam,LMI] = projectionConstructClosedLoopLmi(closedLoopMatrices,genplant,synopts)
%PROJECTIONCONSTRUCTCLOSEDLOOPLMI Constructs the synthesis LMIs that are
%used to solve the closed-loop controller synthesis problem for the
%projection method.
%
%   Syntax:
%       [gam,LMI,var] = projectionConstructClosedLoopLmi(...
%                           closedLoopMatrices,genplant,synopts)
%
%   Inputs:
%       closedLoopMatrices (struct): Structure containing closed-loop 
%           matrices. See <a href=
%           "matlab:help projectionGetClosedloopMatrices"
%           >projectionGetClosedloopMatrices</a>.
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help lpvsynGeneralizedPlantStructure">lpvsynGeneralizedPlantStructure</a>.
%       synopts (struct): Synthesis options structure. See <a href=
%           "matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%   Outputs:
%       gam: Closed-loop gain objective variable as sdpvar, empty in case 
%           passivity.
%       LMI: LMI array containing the LMI constraints for the closed-loop
%           synthesis problem.
%
%   See also LPVSYN.
%

%% Variablles
nx = genplant.nx;
nw = genplant.nw;
nz = genplant.nz;
isCT = genplant.isCT;

perfType = synopts.performance;
numAcc = synopts.numericalAccuracy;

%% LMIs
LMI = [];

% Extract closed-loop matrix variables
Acl = closedLoopMatrices.Acl;
Bcl = closedLoopMatrices.Bcl;
Ccl = closedLoopMatrices.Ccl;
Dcl = closedLoopMatrices.Dcl;
Xcl = closedLoopMatrices.Xcl;

if isCT
    %% Continuous-time
    % CT specific matrices
    Xcldot = closedLoopMatrices.Xcldot;

    switch perfType
        case 'l2'
            gam = sdpvar(1);
    
            H1 = [Acl'*Xcl+Xcl*Acl+Xcldot, Xcl*Bcl,      Ccl';
                  Bcl'*Xcl,                -gam*eye(nw), Dcl';
                  Ccl,                     Dcl,          -gam*eye(nz)];
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(2*nx+nw+nz)];
        case 'passive'
            gam = [];
    
            H1 = [Acl'*Xcl+Xcl*Acl+Xcldot, Xcl*Bcl-Ccl';
                  Bcl'*Xcl-Ccl,            -Dcl'-Dcl];
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(2*nx+nz)];
        case 'h2'
            gam = sdpvar(1);
    
            H1 = [Acl'*Xcl+Xcl*Acl+Xcldot, Xcl*Bcl;
                  Bcl'*Xcl,                -gam*eye(nw)];
            H2 = [Xcl, Ccl';
                  Ccl, gam*eye(nz)];
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(2*nx+nw)];
            LMI = [LMI; (H2+H2')*.5>=numAcc*eye(2*nx+nz)];
        case 'l1'
            gam = sdpvar(1);
            mu = rolmipvar('mu','scalar');
            lambda = rolmipvar('lambda','scalar');
            
            H1 = [Acl'*Xcl+Xcl*Acl+Xcl*lambda+Xcldot, Xcl*Bcl;
                  Bcl'*Xcl,                           -mu*eye(nw)];
            H2 = [Xcl*lambda,  zeros(2*nx,nw),    Ccl';
                  rz(nw,2*nx), -(mu-gam)*eye(nw), Dcl';
                  Ccl,         Dcl,               gam*eye(nz)];
            LMI = [LMI;(H1+H1')*.5 <= -numAcc*eye(nw+2*nx)];
            LMI = [LMI;(H2+H2')*.5 >= numAcc*eye(nw+2*nx+nz)];
    end
else
    %% Discrete-time
    % DT specific matrices
    XclPlus = closedLoopMatrices.XclPlus;
    Gcl = closedLoopMatrices.Gcl;
    
    switch perfType
        case 'l2'
            gam = sdpvar(1);
            
            H1 = [XclPlus,        Acl*Gcl,        Bcl,            zeros(2*nx,nz);
                  Gcl'*Acl',      Gcl+Gcl'-Xcl,   zeros(2*nx,nw), Gcl'*Ccl';
                  Bcl',           zeros(nw,2*nx), gam*eye(nw),    Dcl';
                  zeros(nz,2*nx), Ccl*Gcl,        Dcl,            gam*eye(nz)];
            LMI = [LMI; (H1+H1')*.5>=numAcc*eye(4*nx+nw+nz)];
        case 'passive'
            error(['Not (yet) available, use transformation method. ',...
                'See help lpvsynOptions.']);
        case 'h2'
            error(['Not (yet) available, use transformation method. ',...
                'See help lpvsynOptions.']);
    end
end
end

%% Create zero matrix as rolmipvariable
function out = rz(n,m)
    out = rolmipvar(1,1,'z',0,0)*zeros(n,m);
end