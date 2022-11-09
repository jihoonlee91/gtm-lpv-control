function [K,gam,Xcl] = lpvsyn(P,ny,nu,synopts)
%LPVSYN Suboptimal state-space affine LPV controller synthesis. 
%
%   Syntax:
%       [K, gam, Xcl] = lpvsyn(P, ny, nu, ...)
%
%       [K, gam, Xcl] = lpvsyn(P, ny, nu, synopts)
%
%   Inputs:
%       P (lpvss): LPV state-space system with affine parameter depenency
%           for which the controller is synthesized.
%       ny (double): The number of measured variables, i.e. the last ny 
%           outputs of the plant P used as inputs to the controller K.
%       nu (double): The number of controlled variables, i.e. the last nu
%           inputs of the plant P that are connected to the output of the
%           controller K.
%       type (char): Type of optimal controller that is synthesized, 
%           supported types are: 'l1', 'l2', 'h2' and 'passive'. If type is
%           not given 'l2' is used.
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
%References:
%   Apkarian, P., Gahinet, P., and G. Becker, "Self-Scheduled Hinf Control 
%   of Linear Parameter-Varying Systems: A design example," Automatica, 
%   (1995).
%
%   Apkarian, P., and Adams, R.J., "Advanced gain-scheduling techniques for 
%   uncertain systems," IEEE Transactions on Automatic Control, (1998).
%
%   Gahinet, P., & Apkarian, P., "A linear matrix inequality approach to H∞
%   control," International Journal of Robust and Nonlinear Control, 
%   (1994).
%
%   Ali, M., and Werner, H., "Discrete-Time LPV Controller Synthesis Using 
%   Dilated LMIs with Application to an Arm-Driven Inverted Pendulum," 
%   Proc. of 18th World Congress IFAC, (2011).
%
%   De Caigny, J., Camino, J.F., Oliveira, R. C. L. F., Peres, P. L. D., 
%   and Swevers J., "Gain-scheduled dynamic output feedback control for 
%   discrete-time LPV systems," International Journal of Robust and 
%   Nonlinear Control, (2012).
%
%   Scherer, C. W. and Weiland, S., "Linear Matrix Inequalities in Control,
%   " Jan-2015. <a 
%   href="https://www.imng.uni-stuttgart.de/mst/files/LectureNotes.pdf"
%   >[Online]</a>.  
%

%% Input parser
% Check if synthesis options are given
if ~exist('synopts','var')
    synopts = lpvsynOptions;
end

%% Checks
assert(isa(P,'lpvss'),'System is not an lpvss object.');
assert(checkNyNu(ny,nu),'Inputs ny and/or nu are not integer valued or not positive.');
assert(all([isaffine(P.A),isaffine(P.B),isaffine(P.C),isaffine(P.D)]),...
       'System is not affinely dependent on the parameter.');

%% Obtain structure with all variables of generalized plant
verbose = synopts.verbose;

if verbose; disp('Converting system to correct format...'); end
[genplant,synopts] = lpvsynGeneralizedPlantStructure(P,ny,nu,synopts);

%% Variables
nw = genplant.nw;
nz = genplant.nz;
isCT = (genplant.Ts == 0);

perfType = synopts.performance;
backoff = synopts.backoff;
solOpt = synopts.solverOptions;
method = synopts.method;
parVarStorage = synopts.parameterVaryingStorage;
imprCond = synopts.improveConditioning;

%% Performance specific checks
switch perfType
    case 'h2'
        assert(verifyDclZero(genplant),'D-matrix of the closed-loop is not zero! See the help of this function for more information');
    case 'passive'
        assert(nw == nz,'Passivity based synthesis not possible, number of inputs and outputs are not the same');
end

%% Method specific warnings/checks
% Use projection method in case of parameter-varying storage function
if isCT && parVarStorage && strcmp(method,'transformation')
    warning(['Parameter-varying storage function not supported for ',...
        'transformation method for continuous-time systems. Switching ',...
        'to projection method.']);
    method = 'projection';
    synopts.method = method;
end

% Give warning for DT projection method with parameter-varying storage
if ~isCT && parVarStorage && strcmp(method,'projection')
    warning(['Parameter-varying storage function using the ',...
             'projection method for discrete-time systems might be ',...
             'infeasible. Try transformation method in that case. ',...
             'See help lpvsynOptions.']);
end

%% Synthesis
switch method
    %% Transformation method
    case 'transformation'
        % Apply NL transformation
        if verbose; disp('Applying NL transformation...'); end
        [matrices,mapping] = transformationApply(genplant,synopts);
        
        % Construct the synthesis LMIs
        if verbose; disp('Setting up transformed synthesis LMI...'); end
        [gam,LMI,var] = transformationConstructSynthesisLmi(matrices,genplant,synopts);
        
        % Add pole constraints
        LMI = lpvsynPoleConstraint(LMI,matrices.Xt,matrices.At,genplant,'standard',synopts);
        
        % Solve synthesis LMIs
        if verbose; disp('Solving transformed synthesis LMI...'); end
        [diagnostics,LMI,gamValue] = lpvsynSolveLmi(LMI,gam,var,synopts);

        LMI = lpvsynCheckLmiProblems(diagnostics,LMI,gam,gamValue,synopts,0);
        gam = gamValue*backoff;

        if verbose
            if ~isempty(gam)
                fprintf('Transformed synthesis LMI solved with gamma = %.4g\n',gam); 
            else
                disp('Transformed synthesis LMI solved'); 
            end
        end
        
        % Numerical conditioning
        if imprCond
            % Better conditioning (1/2)
            [alph,LMI] = lpvsynConditioningLMI(LMI,mapping,genplant,...
                synopts,ny,nu,'transformation-1');
            
            if verbose; disp('Improving conditioning of solution (1/2)...'); end
            diagnostics = optimize(LMI,alph,solOpt);
    
            LMI = lpvsynCheckLmiProblems(diagnostics,LMI,alph,value(alph),synopts,0);
            
            % Better conditioning (2/2)
            [bet,LMI] = lpvsynConditioningLMI(LMI,mapping,genplant,...
                synopts,ny,nu,'transformation-2');
            
            if verbose; disp('Improving conditioning of solution (2/2)...'); end
            diagnostics = optimize(LMI,-bet,solOpt);
    
            lpvsynCheckLmiProblems(diagnostics,LMI,bet,value(bet),synopts,1);
        elseif ~isempty(gam)
            % Reformulate LMI with (fixed) backed-off gamma
            if verbose; disp('Solving synthesis LMI with backed-off gamma...'); end
            diagnostics = optimize(LMI,[],solOpt);
            assert(diagnostics.problem == 0,lpvsynErrorMessage(diagnostics));
        end
        
        % Construct controller
        if verbose; disp('Constructing controller...'); end
        [K,Xcl] = transformationConstructController(mapping,genplant);

    %% Projection method
    case 'projection'
        % Apply projection
        if verbose; disp('Applying projection...'); end
        [matrices,mapping] = projectionApply(genplant,synopts);
        
        % Construct the synthesis (existence) LMIs
        if verbose; disp('Setting up projected synthesis LMI...'); end
        [gam,LMI,var] = projectionConstructSynthesisLmi(matrices,mapping,genplant,synopts);

        % Add pole constraints
        % TODO (26/11/2021): These still give inconsistent results
        LMI = lpvsynPoleConstraint(LMI,matrices.Xr,matrices.Ar.Full,genplant,'projected',synopts);
        
        % Solve synthesis LMIs
        if verbose; disp('Solving projected synthesis LMI...'); end
        [diagnostics,LMI,gamValue] = lpvsynSolveLmi(LMI,gam,var,synopts);

        LMI = lpvsynCheckLmiProblems(diagnostics,LMI,gam,gamValue,synopts,0);
        gam = gamValue*backoff;

        if verbose
            if ~isempty(gam)
                fprintf('Projected synthesis LMI solved with gamma = %.4g\n',gam); 
            else
                disp('Projected synthesis LMI solved'); 
            end
        end

        % Numerical conditioning
        % TODO: (01/12/2021): Sometimes results in numerical issues for
        % solving controller LMI, not sure why (so disabled for now)
%         if imprCond
%             % Better conditioning (1/2)
%             [alph,LMI] = lpvsynConditioningLMI(LMI,mapping,genplant,...
%                 synopts,ny,nu,'projection-1');
%             
%             if verbose; disp('Improving conditioning of solution (1/2)...'); end
%             diagnostics = optimize(LMI,alph,solOpt);
% 
%             LMI = lpvsynCheckLmiProblems(diagnostics,LMI,alph,value(alph),synopts,0);
%             
%             % Better conditioning (2/2)
%             [bet,LMI] = lpvsynConditioningLMI(LMI,mapping,genplant,...
%                 synopts,ny,nu,'projection-2');
%             
%             if verbose; disp('Improving conditioning of solution (2/2)...'); end
%             diagnostics = optimize(LMI,-bet,solOpt);
%     
%             lpvsynCheckLmiProblems(diagnostics,LMI,bet,value(bet),synopts,1);
%         elseif ~isempty(gam)
%             % Reformulate LMI with (fixed) backed-off gamma
%             if verbose; disp('Solving synthesis LMI with backed-off gamma...'); end
%             diagnostics = optimize(LMI,[],solOpt);
%             assert(diagnostics.problem == 0,lpvsynErrorMessage(diagnostics));
%         end
        if ~isempty(gam)
            % Reformulate LMI with (fixed) backed-off gamma
            if verbose; disp('Solving synthesis LMI with backed-off gamma...'); end
            diagnostics = optimize(LMI,[],solOpt);
            assert(diagnostics.problem == 0,lpvsynErrorMessage(diagnostics));
        end

        % Get closed-loop matrices
        if verbose; disp('Constructing closed-loop matrices...'); end
        closedLoopMatrices = projectionGetClosedloopMatrices(genplant,mapping);
        
        % Construct the synthesis (controller) LMIs
        if verbose; disp('Setting up controller LMI...'); end
        [gam,LMI] = projectionConstructClosedLoopLmi(closedLoopMatrices,genplant,synopts);

        % Add pole constraints
        % TODO (26/11/2021): These still give inconsistent results
        LMI = lpvsynPoleConstraint(LMI,closedLoopMatrices.Xcl,...
            closedLoopMatrices.Xcl*closedLoopMatrices.Acl,genplant,...
            'standard',synopts);

        % Solve synthesis LMIs
        if verbose; disp('Solving controller LMI...'); end
        diagnostics = optimize(LMI,gam,solOpt);

        LMI = lpvsynCheckLmiProblems(diagnostics,LMI,gam,value(gam),synopts,0);
        gam = value(gam);

        if verbose
            if ~isempty(gam)
                fprintf('Controller synthesis solved with gamma = %.4g\n',gam); 
            else
                disp('Controller synthesis solved'); 
            end
        end

        % Better conditioning
        if imprCond
            [alph,LMI] = lpvsynConditioningLMI(LMI,mapping,genplant,...
                synopts,ny,nu,'projection-3');
            
            if verbose; disp('Improving conditioning of controller solution...'); end
            diagnostics = optimize(LMI,alph,solOpt);
    
            lpvsynCheckLmiProblems(diagnostics,LMI,alph,value(alph),synopts,0);
        end
        
        % Construct controller
        if verbose; disp('Constructing controller...'); end
        [K,Xcl] = projectionConstructController(genplant,mapping,closedLoopMatrices.Xcl);
end
if verbose; disp('Done'); end
end

%% Local functions
%% Checks if ny, nu are positive integers
function out = checkNyNu(ny,nu)
% check if it is a whole number
check1 = (round(ny,0) == ny) & (round(nu,0) == nu); 
% check if they are positive
check2 = (ny > 0) & (nu > 0); 
out = check1 & check2;
end

%% Verifies whether the closed loop matrix D_cl is zero (used for H2)
function flag = verifyDclZero(gp) 
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