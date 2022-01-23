function [K,gam,Xcl] = lpvh2syn(P,ny,nu,synopts)
%LPVH2SYN Generalized H2 suboptimal state-space affine LPV controller 
%synthesis. 
%
%   Syntax:
%       [K, gam, Xcl] = lpvsyn(P, ny, nu, synopts)
%
%   Inputs:
%       P (lpvss): LPV state-space system with affine parameter depenency
%           for which the controller is synthesized.
%       ny (int): The number of measured variables, i.e. the last ny 
%           outputs of the plant P used as inputs to the controller K.
%       nu (int): The number of controlled variables, i.e. the last nu
%           inputs of the plant P that are connected to the output of the
%           controller K.
%       synopts (structure): Stucture containing LPV synthesis options, see
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%	Outputs:
%       K: Suboptimal state-space affine controller as lpvss object.
%       gam: Upperbound for the gain of the closed-loop system.
%       Xcl: Matrix of closed-loop storage/Lyapunov function, i.e. the
%           Lypunov function is of the form V(x) = x'*Xcl*x. Either 
%           constant or parameter dependent.
%

%% Convert LPVCore generalized plant to rolmip variables
genplant = generalizedPlantToRolmip(P,ny,nu);

% Check whether D_cl is zero
assert(verifyDclZero(genplant),'D-matrix of the closed-loop is not zero! See the help of this function for more information');

%% Variables
nx = genplant.nx;
nw = genplant.nw;
nz = genplant.nz;

backoff = synopts.backoff;
numAcc = synopts.numericalAccuracy;
solOpt = synopts.solverOptions;

%% Synthesis
%% Continuous time
if genplant.Ts == 0
    LMI = [];
    
    % H2 controller synthesis is for now based on a NL transformation
    disp('Applying NL transformation...')
    [Xt,At,Bt,Ct,~,mapping] = applyNonlinearLmiTransformationCT(genplant,synopts);
    
    % Construct the synthesis LMI's
    disp('Setting up transformed synthesis LMI...')
    gam = sdpvar(1);
    H1 = [At'+At, Bt;
          Bt',    -gam*eye(nw)];
    H2 = [Xt, Ct';
          Ct, gam*eye(nz)];
    LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(2*nx+nw)];
    LMI = [LMI; (H2+H2')*.5>=numAcc*eye(2*nx+nz)];
    LMI = [LMI; (Xt+Xt')*.5>=numAcc*eye(2*nx)];
    
    % Solve synthesis LMI's
    disp('Solving transformed synthesis LMI...')
    diagnostics = optimize(LMI,gam,solOpt);
    
    if diagnostics.problem == 1                 % Don't use assert here as solver may report
        error('Controller synthesis failed');   % problems, but solution might still be close
    end                                         % to optimal and can be found with fixed gamma. 
    
    gamValue = value(gam);
        
    fprintf('Transformed synthesis LMI solved with gamma = %.4g\n',gamValue)
    gamValue = gamValue*backoff;

    % Reformulate LMI with (fixed) backed-off gamma for better 
    % numerical conditioning
    fprintf('Setting up synthesis LMI with fixed gamma = %.4g\n',gamValue)
    LMI = replace(LMI,gam,gamValue);

    diagnostics = optimize(LMI,[],solOpt);
    assert(diagnostics.problem == 0,'Controller synthesis failed');
    fprintf('Transformed synthesis LMI solved with gamma = %.4g\n',gamValue)

    gam = gamValue;
    
    % Construct controller
    disp('Constructing controller...')
    [K,Xcl] = nonlinTransformConstructControllerCT(mapping,genplant,synopts);
%% Discrete time
else
    % Used fixed storage/Lyapunov function
    % Apply NL transformation
    [Xt,Gt,At,Bt,Ct,~,mapping] = applyNonlinearLmiTransformationDT(genplant,synopts);
    
    % Construct the synthesis LMI's
    disp('Setting up transformed synthesis LMI...')
    gam = sdpvar(1);
    H1 = [Xt,  At,             Bt;
          At', Gt,             zeros(2*nx,nw);
          Bt', zeros(nw,2*nx), gam*eye(nw)];
    H2 = [Gt, Ct';
          Ct, gam*eye(nz)];
    LMI = [(H1+H1')*.5>=numAcc*eye(4*nx+nw);
           (H2+H2')*.5>=numAcc*eye(2*nx+nz)];

    % Solve synthesis LMI's
    disp('Solving transformed synthesis LMI...')
    diagnostics = optimize(LMI,gam,solOpt);
    
    if diagnostics.problem == 1                 % Don't use assert here as solver may report
        error('Controller synthesis failed');   % problems, but solution might still be close
    end                                         % to optimal and can be found with fixed gamma. 
    
    gamValue = value(gam);
        
    fprintf('Transformed synthesis LMI solved with gamma = %.4g\n',gamValue)
    gamValue = gamValue*backoff;

    % Reformulate LMI with (fixed) backed-off gamma for better 
    % numerical conditioning
    fprintf('Setting up synthesis LMI with fixed gamma = %.4g\n',gamValue)
    LMI = replace(LMI,gam,gamValue);

    diagnostics = optimize(LMI,[],solOpt);
    assert(diagnostics.problem == 0,'Controller synthesis failed');
    fprintf('Transformed synthesis LMI solved with gamma = %.4g\n',gamValue)

    gam = gamValue;

    % Construct controller
    disp('Constructing controller...')
    [K,Xcl] = nonlinTransformConstructControllerDT(mapping,genplant,synopts);
end
end

%% Local functions
function flag = verifyDclZero(gp) % Verifies whether the closed loop matrix D_cl is zero.
    % First verify if Dzw = 0
    if isa(gp.Dzw,'rolmipvar')
        flag1 = all(abs(cell2mat(coeffs(gp.Dzw)))<=2*eps, 'all');
    else
        flag1 = all(abs(gp.Dzw)<=2*eps, 'all');
    end
    % Verify if Dzu or Dyw is zero
    flag2 = (all(abs(gp.Dzu)<=2*eps, 'all') || all(abs(gp.Dyw)<=2*eps, 'all'));
    % To avoid algebraic constraint, Dzw must be zero and (Dzu or Dyw) must be
    % zero
    flag = flag1 && flag2;
end