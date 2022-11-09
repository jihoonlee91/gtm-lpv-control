function [gam,LMI,Xt,var] = lpvnormConstructAnalysisLmi(matrices,P,stStruct,opt)
%LPVNORMCONSTRUCTANALYSISLMI Constructs the analysis LMIs that are used to
%solve the LPV analysis problem.
%
%   Syntax:
%       [gam,LMI,Xt,var] = lpvnormConstructAnalysisLmi(matrices,P,...
%                                                      stStruct,opt)
%
%   Inputs:
%       matrices (struct): Structure containing analysis matrices.
%       P (lpvss): LPVSS object of the plant which is analyzed.
%       stStruct (struct): Internal structure variable with fields for
%           constructing the storage function with the correct parameter
%           dependency.
%       opt (struct): Structure used with internal settings for <a href=
%           "matlab:help lpvnorm">lpvnorm</a>.
%
%   Outputs:
%       gam: Closed-loop gain objective variable as sdpvar, empty in case 
%           passivity.
%       LMI: LMI array containing the LMI constraints for the transformed
%           synthesis problem.
%       Xt: Rolmip matrix corresponding to the storage/Lyapunov function.
%       var: Structure containing other optimization variables. (Only used
%           for Linf based analysis).
%
%   See also LPVNORM.
%

%% Variablles
nx = P.Nx;
nu = P.Nu;
ny = P.Ny;

perfType = opt.type;
sysType = opt.sysType;
numAcc = opt.NumericalAccuracy;
paramVarStorage = opt.ParameterVaryingStorage;
isCT = opt.isCT;

switch sysType
    case 'affine'
        stDep = stStruct.stDep;
        rhoRange = stStruct.rhoRange;
        rhoRateBoundDummy = stStruct.rhoRateBoundDummy;
end

At = matrices.At;
Bt = matrices.Bt;
Ct = matrices.Ct;
Dt = matrices.Dt;
%% LMIs
LMI = [];
var = [];

if isCT
    %% Continuous-time
    if paramVarStorage == 0
        switch sysType
            case 'affine'
                Xt = rolmipvar(nx,nx,'X','sym',0,0);
            case 'grid'
                Xt = gridvar(sdpvar(nx,nx,'sym'));
        end
        XtDot = zeros(nx);
    else
        switch sysType
            case 'affine'
                Xt = rolmipvar(nx,nx,'X','sym',stDep,rhoRange);
                XtDot = diff(Xt,'XtDot',rhoRange,rhoRateBoundDummy);
            case 'grid'
                error('TBD');
        end
    end

    switch perfType
        case 'l2'
            gam = sdpvar(1);
    
            H1 = [At'*Xt+Xt*At+XtDot, Xt*Bt,        Ct';
                  Bt'*Xt,             -gam*eye(nu), Dt';
                  Ct,                 Dt,           -gam*eye(ny)];
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nx+nu+ny)];
            LMI = [LMI; Xt>=numAcc*eye(nx)];
        case 'passive'
            gam = [];
    
            H1 = [At'*Xt+Xt*At+XtDot, Xt*Bt-Ct';
                          Bt'*Xt-Ct,          -Dt'-Dt];
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nx+nu)];
            LMI = [LMI; Xt>=numAcc*eye(nx)];
        case 'h2'
            gam = sdpvar(1);

            H1 = [At'*Xt+Xt*At+XtDot, Xt*Bt;
                  Bt'*Xt,             -gam*eye(nu)];
            H2 = [Xt, Ct';
                  Ct, gam*eye(ny)];
            LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(nx+nu)];
            LMI = [LMI; (H2+H2')*.5>=numAcc*eye(nx+ny)];
        case 'linf'
            gam = sdpvar(1);
            switch sysType
                case 'affine'
                    mu = rolmipvar('mu','scalar');
                case 'grid'
                    mu = gridvar(sdpvar(1));
            end
            lambda = sdpvar(1);
            
            H1 = [At'*Xt+Xt*At+Xt*lambda+XtDot, Xt*Bt;
                  Bt'*Xt,                       -mu*eye(nu)];
            H2 = [Xt*lambda,    zeros(nx,nu),      Ct';
                  zeros(nu,nx), -(mu-gam)*eye(nu), Dt';
                  Ct,           Dt,                gam*eye(ny)];
            LMI = [(H1+H1')*.5 <= -numAcc*eye(nu+nx);
                   (H2+H2')*.5 >= numAcc*eye(nu+nx+ny)];

            var.mu = mu;
            var.lambda = lambda;
    end
else
    %% Discrete-time
    if paramVarStorage == 0
        switch sysType
            case 'affine'
                Xt = rolmipvar(nx,nx,'X','sym',0,0);
                Gt = rolmipvar(nx,nx,'G','full',0,0);
            case 'grid'
                Xt = gridvar(sdpvar(nx,nx,'sym'));
                Gt = gridvar(sdpvar(nx,nx,'full'));
        end
        XtDelta = zeros(nx);
    else
        switch sysType
            case 'affine'
                Xt = rolmipvar(nx,nx,'X','sym',stDep,rhoRange);
                % TODO (2021/09/27): Using diff in DT okay for affine dep. not 
                % for polynomial etc.!
                XtDelta = diff(Xt,'XtDelta',rhoRange,rhoRateBoundDummy);
                Gt = rolmipvar(nx,nx,'G','full',stDep,rhoRange);
            case 'grid'
                error('TBD');
        end
    end
    XtPlus = Xt+XtDelta;

    switch perfType
        case 'l2'
            gam = sdpvar(1);
    
            H1 = [XtPlus,       At*Gt,        Bt,           zeros(nx,ny);
                  Gt'*At',      Gt+Gt'-Xt,    zeros(nx,nu), Gt'*Ct';
                  Bt',          zeros(nu,nx), gam*eye(nu),  Dt';
                  zeros(ny,nx), Ct*Gt,        Dt,           gam*eye(ny)];
            LMI = [LMI; H1>=numAcc*eye(2*nx+nu+ny)];
        case 'passive'
            gam = [];
    
            H1 = [XtPlus,  At*Gt,     Bt;
                  Gt'*At', Gt+Gt'-Xt, Gt'*Ct';
                  Bt',     Ct*Gt,     Dt+Dt'];
            LMI = [LMI; (H1+H1')*.5>=numAcc*eye(2*nx+nu)];
        case 'h2'
            gam = sdpvar(1);
            
            H1 = [XtPlus,  At*Gt,        Bt;
                  Gt'*At', Gt+Gt'-Xt,    zeros(nx,nu);
                  Bt',     zeros(nu,nx), gam*eye(nu)];
            H2 = [Gt+Gt'-Xt, Gt'*Ct';
                  Ct*Gt,     gam*eye(ny)];
            LMI = [LMI; (H1+H1')*.5>=numAcc*eye(2*nx+nu)];
            LMI = [LMI; (H2+H2')*.5>=numAcc*eye(nx+ny)];
        case 'linf'
            gam = sdpvar(1);
            switch sysType
                case 'affine'
                    mu = rolmipvar('mu','scalar');
                case 'grid'
                    mu = gridvar(sdpvar(1));
            end
            lambda = sdpvar(1);

            H1 = [XtPlus,  At*Gt,                  Bt;
                  Gt'*At', (Gt+Gt'-Xt)*(1-lambda), zeros(nx,nu);
                  Bt',     zeros(nu,nx),           mu*eye(nu)];
            H2 = [(Gt+Gt'-Xt)*lambda, zeros(nx,nu),      Gt'*Ct';
                  zeros(nu,nx),       (-mu+gam)*eye(nu), Dt';
                  Ct*Gt,              Dt,                gam*eye(ny)];
            LMI = [(H1+H1')*.5 >= numAcc*eye(2*nx+nu);
                   (H2+H2')*.5 >= numAcc*eye(nu+nx+ny)];

            var.mu = mu;
            var.lambda = lambda;
    end
end
end
