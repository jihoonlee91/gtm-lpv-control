function [K,gam,Xcl] = lpvl2syn(P,ny,nu,synopts)
%LPVL2SYN L2-gain suboptimal state-space affine LPV controller synthesis. 
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

%% Variables
nx = genplant.nx;
nw = genplant.nw;
nz = genplant.nz;

backoff = synopts.backoff;
numAcc = synopts.numericalAccuracy;
solOpt = synopts.solverOptions;
l2Method = synopts.l2Method;

%% Synthesis
%% Continuous time
if genplant.Ts == 0
    LMI = [];
    % Reparametrization method
    if l2Method
        % Note: For now only for fixed lyapunov function
        % Apply NL transformation
        disp('Applying NL transformation...')
        [Xt,At,Bt,Ct,Dt,mapping] = applyNonlinearLmiTransformationCT(genplant,synopts);
        
        % Construct the synthesis LMIs
        disp('Setting up transformed synthesis LMI...')
        gam = sdpvar(1);
        H1 = [At'+At, Bt,           Ct';
              Bt',    -gam*eye(nw), Dt';
              Ct,     Dt,           -gam*eye(nz)];
        LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(2*nx+nw+nz)];
        LMI = [LMI; (Xt+Xt')*.5>=numAcc*eye(2*nx)];
        
        % Solve synthesis LMIs
        disp('Solving transformed synthesis LMI...')
        diagnostics = optimize(LMI,gam,solOpt);
        
        if diagnostics.problem == 1                 % Don't use assert here to check if YALMIP returns 0,
            error('Controller synthesis failed');   % as solver may report problems, but solution might still
        end                                         % be close to optimal and can be found with fixed gamma.
        
        gamValue = value(gam);
        
        fprintf('Transformed synthesis LMI solved with gamma = %.4g\n',gamValue)
        gamValue = gamValue*synopts.backoff;
        
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
    else
        % Projection
        [K,gam,Xcl] = projectionMethod(genplant,backoff,numAcc,solOpt);
    end
%% Discrete time
else
    if l2Method
        % Used fixed storage/Lyapunov function
        % Apply NL transformation
        [Xt,Gt,At,Bt,Ct,Dt,mapping] = applyNonlinearLmiTransformationDT(genplant,synopts);
        
        % Construct the synthesis LMI's
        disp('Setting up transformed synthesis LMI...')
        gam = sdpvar(1);
        H1 = [Xt,             At,             Bt,             zeros(2*nx,nz);
              At',            Gt,             zeros(2*nx,nw), Ct';
              Bt',            zeros(nw,2*nx), gam*eye(nw),    Dt';
              zeros(nz,2*nx), Ct,             Dt,             gam*eye(nz)];
        LMI = (H1+H1')*.5>=numAcc*eye(4*nx+nw+nz);
        
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
    else
        error('Projection method is not available (yet) for discrete time systems');
        % TODO (04/08/20): Find out why this is not working for DT
%         [K,gamma,Theta,Xcl] = projectionMethod(genplant,synopts);
    end
end
end

%% Projection method
% Apkarian, P., Gahinet, P., and G. Becker, "Self-Scheduled H∞ Control of 
% Linear Parameter-Varying Systems: A design example," Automatica, (1995).
%
% Gahinet, P., & Apkarian, P., "A linear matrix inequality approach to H∞
% control," International Journal of Robust and Nonlinear Control, (1994).

function [K,gam,Xcl] = projectionMethod(genplant,backoff,numAcc,solOpt)
% Solve LMI with controller variables elminitated
disp('Setting projection LMI...');
[LMI,gam,X,Xd,Y,~] = projectionConstructExistenceLmi(genplant,numAcc);

disp('Solving projection LMI...');
diagnostics = optimize(LMI,gam,solOpt);

gamValue = value(gam);

if diagnostics.problem == 1
    error('Controller synthesis failed');
end

fprintf('Projection LMI solved, gamma = %.4g\n',gamValue);

% Solve existince LMI fixed gamma
disp('Setting up projection LMI with fixed gamma...');

gamValue = gamValue*backoff;

LMI = replace(LMI,gam,gamValue);

disp('Solving projection LMI with fixed gamma...');
diagnostics = optimize(LMI,[],solOpt);

gam = gamValue;

assert(diagnostics.problem == 0,'Controller synthesis failed (existence LMI - fixed gamma)');
fprintf('Projection LMI with fixed gamma solved, gamma = %.4g\n',gam);
Xcl = value(X);

% Controller LMI
disp('Setting up controller construction LMI...');
[LMI,Theta] = projectionConstructControllerLmi(genplant,X,Xd,Y,gam,numAcc);

disp('Solving controller construction LMI...');
diagnostics = optimize(LMI,[],solOpt);

assert(diagnostics.problem == 0,sprintf('Controller synthesis failed (matrix construction), gamma: %.4g',gam));
fprintf('Controller construction LMI solved, gamma = %.4g\n',gam);

% Construct controller
disp('Constructing controller...');
K = projectionConstructController(genplant,Theta);
disp('Controller construction done!');
end