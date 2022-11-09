function [out,X] = lpvnormSolveLmi(LMI,gam,Xt,var,stStruct,P,opt)
%LPVNORMSOLVELMI Solves the LMI for the LPV analysis problem.
%
%   Syntax:
%       [out,X] = lpvnormSolveLmi(LMI,gam,Xt,var,stStruct,P,opt)
%
%   Inputs:
%       LMI (lmi): YALMIP LMI array containing LMI constraints for the
%           optimization problem.
%       gam (sdpvar): YALMIP sdpvar corresponding to the closed-loop gain 
%           objective variable, empty in case of passivity.
%       Xt (rolmip): Rolmipvar matrix corresponding to the storage/Lyapunov 
%           function.
%       var (struct): Structure containing other optimization variables. 
%           (Only used for Linf based analysis).
%       stStruct (struct): Internal structure variable with fields for
%           constructing the storage function with the correct parameter
%           dependency.
%       P (lpvss):LPVSS object of the plant which is analyzed.
%       opt (struct): Structure used with internal settings for <a href=
%           "matlab:help lpvnorm">lpvnorm</a>.
%
%   Outputs:
%       out: Value of the norm in case of the 'linf', 'l2' or 'h2' norm or
%           true in the case of a passivity check and the system is passive.
%       X: Value of quadratic storage function matrix as pmatrix object,
%           i.e., value of Xt converted to pmatrix object.
%   
%   See also LPVNORM.
%

%% Variables
type = opt.type;
sysType = opt.sysType;
solOpt = opt.SolverOptions;
linfOpt = opt.LinfOptions;
verbose = opt.Verbose;
paramVarStorage = opt.ParameterVaryingStorage;

switch sysType
    case 'affine'
        rateBoundedRhoIndex = stStruct.rateBoundedRhoIndex;
        stDep = stStruct.stDep;
end


%% Solve LMIs
if verbose; disp('Solving LMI...');end
switch type
    %% Linf analysis
    case 'linf'
        switch sysType
            case 'affine'
                Xv = storageFunctionToAffine(Xt,stDep);
            case 'grid'
                Xv = Xt.matrices;
        end
        
        gamConstraint = sdpvar(1);
        LMI = LMI+(gam>=gamConstraint);
        linfSolve = optimizer(LMI,gam,solOpt,{var.lambda,gamConstraint},[{gam},Xv(:)']);
        [gamValue,~,linfSolution,solverError] = linfLineSearch(linfSolve,linfOpt,verbose,1.01);
        
        if solverError
            out = NaN;
            X = NaN;
        else
            out = gamValue;

            Xv = linfSolution(2:end);
            switch sysType
                case 'affine'
                    X = affineStorageFunctionToPmatrix(Xv,P,...
                            paramVarStorage,rateBoundedRhoIndex);
                case 'grid'
                    Xt.matrices = Xv;
                    X = getMatrices(Xt);
            end
        end
    %% L2, passive and H2 analysis
    otherwise
        diagnostics = optimize(LMI,gam,solOpt);
        gamValue = value(gam);
        [solverError,gamValue] = lpvnormCheckLmiProblems(diagnostics,LMI,gam,gamValue,opt);

        if solverError
            out = NaN;
            X = NaN;
        else
            if strcmp(type,'passive')
                out = true;
            else
                out = gamValue;
            end
            switch sysType
                case 'affine'
                    X = rolmipToPmatrix(Xt,P.A,stDep);
                case 'grid'
                    X = getMatrices(value(Xt));
            end
        end
end
if verbose && ~solverError; disp('Done');end
end

%% Local functions
%% (Pre)compute affine matrices of storage function (only used by linf)
% Workaround for Linf as Yalmips optimizer doesn't support rolmipvars.
% Extract affine matrices from rolmipvar-object of X = X0+X1*p1+X2*p2+...
% Xv{1} = X0; Xv{2} = X1; Xv{3} = X2; ...
function Xv = storageFunctionToAffine(X,stv)
    if isempty(stv)
        Xv = X;
    else
        npX = numel(stv)-1;
        Xv = cell(npX+1,1);
        Xv{1} = evalpar(X,stv{1});
        for i = 2:npX+1
            Xv{i} = evalpar(X,stv{i})-Xv{1};
        end
    end
end

%% Convert affine matrices to pmatrix object (only used by linf)
function X = affineStorageFunctionToPmatrix(Xv,P,paramVarStorage,rateBoundedRhoIndex)
    if paramVarStorage == 0
        X = value(Xv{1});
    else
        %% Create p
        p = cell(P.Np,1);
        At = simplify(P.A); % Use A-matrix to extract the (affine) basis functions
        X = value(Xv{1}); % X0
        for i = 1:numel(rateBoundedRhoIndex)
            pInd = rateBoundedRhoIndex(i);
            p{pInd} = pmatrix(1,At.bfuncs(pInd+1),'SchedulingTimeMap',P.SchedulingTimeMap);
            X = X+value(Xv{i+1})*p{pInd};
        end
    end
end