function [gam,LMI,var] = transformationConstructSynthesisLmi(matrices,genplant,synopts)
%TRANSFORMATIONCONSTRUCTSYNTHESISLMI Constructs the synthesis LMIs that are
%used to solve the transformed controller synthesis problem.
%
%   Syntax:
%       [gam,LMI,var] = transformationConstructSynthesisLmi(matrices,...
%                           genplant,synopts)
%
%   Inputs:
%       matrices (struct): Structure containing closed-loop transformed 
%           matrices. See <a href="matlab:help transformationApply"
%           >transformationApply</a>.
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
%           for Linf based synthesis).
%
%   See also LPVSYN.
%

%% Variablles
nx = genplant.nx;
nw = genplant.nw;
nz = genplant.nz;

perfType = synopts.performance;
numAcc = synopts.numericalAccuracy;
sysType = synopts.systemType;
isCT = genplant.isCT;

%% LMIs
LMI = [];
var = [];

if isCT
    %% Continuous-time
    Xt = matrices.Xt;
    At = matrices.At;
    Bt = matrices.Bt;
    Ct = matrices.Ct;
    Dt = matrices.Dt;

    switch perfType
        case 'l2'
            gam = sdpvar(1);
    
            H1 = [At'+At, Bt,           Ct';
                  Bt',    -gam*eye(nw), Dt';
                  Ct,     Dt,           -gam*eye(nz)];
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(2*nx+nw+nz)];
            LMI = [LMI; (Xt+Xt')*.5>=numAcc*eye(2*nx)];
        case 'passive'
            gam = [];
    
            H1 = [At'+At, Bt-Ct';
                  Bt'-Ct, -Dt'-Dt];
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(2*nx+nz)];
            LMI = [LMI; (Xt+Xt')*.5>=numAcc*eye(2*nx)];
        case 'h2'
            gam = sdpvar(1);
    
            H1 = [At'+At, Bt;
                  Bt',    -gam*eye(nw)];
            H2 = [Xt, Ct';
                  Ct, gam*eye(nz)];
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(2*nx+nw)];
            LMI = [LMI; (H2+H2')*.5>=numAcc*eye(2*nx+nz)];
            LMI = [LMI; (Xt+Xt')*.5>=numAcc*eye(2*nx)];
        case 'linf'
            gam = sdpvar(1);
            switch sysType
                case 'affine'
                    mu = rolmipvar('mu','scalar');  % workaround for bug rolmip
                    var.mu = coefs(mu);
                case 'grid'
                    var.mu = sdpvar(1);
                    mu = gridvar(var.mu);
            end
            lambda = sdpvar(1);
            var.lambda = lambda;

            H1 = [At'+At+Xt*lambda, Bt;
                  Bt',              -mu*eye(nw)];
            H2 = [Xt*lambda,      zeros(2*nx,nw),    Ct';
                  zeros(nw,2*nx), -(mu-gam)*eye(nw), Dt';
                  Ct,             Dt,                gam*eye(nz)];
            LMI = [LMI; (H1+H1')*.5 <= -numAcc*eye(nw+2*nx)];
            LMI = [LMI; (H2+H2')*.5 >= numAcc*eye(nw+2*nx+nz)];
    end
else
    %% Discrete-time
    XtPlus = matrices.XtPlus;
    Xt = matrices.Xt;
    Gt = matrices.Gt;
    At = matrices.At;
    Bt = matrices.Bt;
    Ct = matrices.Ct;
    Dt = matrices.Dt;

    switch perfType
        case 'l2'
            gam = sdpvar(1);
    
            H1 = [XtPlus,         At,             Bt,             zeros(2*nx,nz);
                  At',            Gt+Gt'-Xt,      zeros(2*nx,nw), Ct';
                  Bt',            zeros(nw,2*nx), gam*eye(nw),    Dt';
                  zeros(nz,2*nx), Ct,             Dt,             gam*eye(nz)];
            LMI = [LMI;(H1+H1')*.5>=numAcc*eye(4*nx+nw+nz)];
        case 'passive'
            gam = [];
    
            H1 = [XtPlus,  At,        Bt;
                  At',     Gt+Gt'-Xt, Ct';
                  Bt',     Ct,        Dt+Dt'];
            LMI = [LMI;(H1+H1')*.5>=numAcc*eye(4*nx+nz)];
        case 'h2'
            gam = sdpvar(1);
            
            H1 = [XtPlus,  At,             Bt;
                  At',     Gt+Gt'-Xt,      zeros(2*nx,nw);
                  Bt',     zeros(nw,2*nx), gam*eye(nw)];
            H2 = [Gt+Gt'-Xt, Ct';
                  Ct,        gam*eye(nz)];
            LMI = [LMI; (H1+H1')*.5>=numAcc*eye(4*nx+nw)];
            LMI = [LMI; (H2+H2')*.5>=numAcc*eye(2*nx+nz)];
        case 'linf'
            gam = sdpvar(1);
            switch sysType
                case 'affine'
                    mu = rolmipvar('mu','scalar');  % workaround for bug rolmip
                    var.mu = coefs(mu);
                case 'grid'
                    var.mu = sdpvar(1);
                    mu = gridvar(var.mu);
            end
            lambda = sdpvar(1);
            var.lambda = lambda;

            H1 = [XtPlus, At,                     Bt;
                  At',    (Gt+Gt'-Xt)*(1-lambda), zeros(2*nx,nw);
                  Bt',    zeros(nw,2*nx),         mu*eye(nw)];
            H2 = [(Gt+Gt'-Xt)*lambda, zeros(2*nx,nw),    Ct';
                  zeros(nw,2*nx),     (-mu+gam)*eye(nw), Dt';
                  Ct,                 Dt,                gam*eye(nz)];
            LMI = [LMI; (H1+H1')*.5 >= numAcc*eye(4*nx+nw)];
            LMI = [LMI; (H2+H2')*.5 >= numAcc*eye(nw+2*nx+nz)];

    end
end
end