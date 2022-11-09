function [K,Xcl] = lpvpassyn(P,ny,nu,synopts)
%LPVPASSYN Pasivity based suboptimal state-space affine LPV controller 
%synthesis. 
%
%   Syntax:
%       [K, Xcl] = lpvsyn(P, ny, nu, synopts)
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

numAcc = synopts.numericalAccuracy;
solOpt = synopts.solverOptions;

assert(nw == nz,'Passivity based synthesis not possible, number of inputs and outputs are not the same');

%% Synthesis
%% Continuous time
if genplant.Ts == 0
    LMI = [];

    %% NL Transformation method
    % Note: For now only for fixed lyapunov function

    % Apply NL transformation
    disp('Applying NL transformation...')
    [Xt,At,Bt,Ct,Dt,mapping] = applyNonlinearLmiTransformationCT(genplant,synopts);

    % Construct the synthesis LMI's
    disp('Setting up transformed synthesis LMI...')
    H1 = [At'+At, Bt-Ct';
          Bt'-Ct, -Dt'-Dt];
    LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(2*nx+nz)];
    LMI = [LMI; (Xt+Xt')*.5>=numAcc*eye(2*nx)];

    % Solve synthesis LMI's
    disp('Solving transformed synthesis LMI...')
    diagnostics = optimize(LMI,[],solOpt);

    assert(diagnostics.problem == 0,'Controller synthesis failed');

    disp('Transformed synthesis LMI solved')

    % Construct controller
    disp('Constructing controller...')
    [K,Xcl] = nonlinTransformConstructControllerCT(mapping,genplant,synopts);
%% Discrete time
else
    % Used fixed storage/Lyapunov function
    % Apply NL transformation
    disp('Applying NL transformation...')
    [Xt,Gt,At,Bt,Ct,Dt,mapping] = applyNonlinearLmiTransformationDT(genplant,synopts);

    % Construct the synthesis LMI's
    disp('Setting up transformed synthesis LMI...')
    H1 = [Xt,  At, Bt;
          At', Gt, Ct';
          Bt', Ct, Dt+Dt'];
    LMI = (H1+H1')*.5>=numAcc*eye(4*nx+nz);

    % Solve synthesis LMI's
    disp('Solving transformed synthesis LMI...')
    diagnostics = optimize(LMI,[],solOpt);

    assert(diagnostics.problem == 0,'Controller synthesis failed');

    disp('Transformed synthesis LMI solved')

    % Construct controller
    disp('Constructing controller...')
    [K,Xcl] = nonlinTransformConstructControllerDT(mapping,genplant,synopts);
end
end