function [objective,LMI] = lpvsynConditioningLMI(LMI,mapping,genplant,synopts,ny,nu,constraintType)
%LPVSYNCONDITIONINGLMI Returns objective an LMIs in order to improve
%conditioning of synthesis solutions. Used in the synthesis algorithms.
%
%   Syntax:
%       [objective,LMI] = lpvsynConditioningLMI(LMI,mapping,genplant,...
%                             synopts,ny,nu,systemType,constraintType)
%
%   Inputs:
%       LMI (lmi): YALMIP lmi array containing LMI constraints for the
%           optimization problem.
%       mapping (struct): Structure containing the matrices that resulted
%           from the transformation or projection method. See 
%           <a href="matlab:help transformationApply"
%           >transformationApply</a> and <a href=
%           "matlab:help projectionApply">projectionApply</a>.
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help lpvsynGeneralizedPlantStructure"
%           >lpvsynGeneralizedPlantStructure</a>.
%       synopts (structure): Stucture containing LPV synthesis options, see
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%       ny (int): The number of measured variables, i.e. the last ny 
%           outputs of the plant P used as inputs to the controller K.
%       nu (int): The number of controlled variables, i.e. the last nu
%           inputs of the plant P that are connected to the output of the
%           controller K.
%       constraintType (char): Character array denoting which numerical
%           conditioning LMI is applied. Possible values are
%           'transformation-1', 'transformation-2', 'projection-1', 
%           'projection-2', and 'projection-3'.
%
%   Outputs:
%       objective: Objective variable as sdpvar of the added numerical
%           conditioning constraint.
%       LMI: LMI array containing the new LMI constraints for the
%           optimization problem.
%
%   References:
%       Scherer, C. W. and Weiland, S., "Linear Matrix Inequalities in
%       Control," Remark 4.6, page 95, Jan-2015. <a href=
%       "https://www.imng.uni-stuttgart.de/mst/files/LectureNotes.pdf"
%       >[Online]</a>.  
%
%   See also LPVSYN.
%

%% Variables
nx = genplant.nx;
isCT = genplant.isCT;
numAcc = synopts.numericalAccuracy;

%% Construct constraints
%% Continuous-time
if isCT
    switch constraintType
        case 'transformation-1'
            alph = sdpvar(1);
            H = [zeros(nx),zeros(nx,nu),mapping.K,mapping.L;...
                 zeros(nu,nx),zeros(nu),mapping.M,mapping.N;...
                 mapping.K',mapping.M',zeros(nx),zeros(nx,ny);...
                 mapping.L',mapping.N',zeros(ny,nx),zeros(ny)]+...
                alph*eye(2*nx+nu+ny);
            LMI = [LMI; H>= numAcc*eye(2*nx+nu+ny)];
            LMI = [LMI; mapping.X-alph*eye(nx)<=-numAcc*eye(nx)];
            LMI = [LMI; mapping.Y-alph*eye(nx)<=-numAcc*eye(nx)];

            objective = alph;
        case {'transformation-2','projection-2'}
            bet = sdpvar(1); assign(bet,1);
            H = [mapping.Y,zeros(nx);
                 zeros(nx),mapping.X]+...
                [zeros(nx),bet*eye(nx);
                 bet*eye(nx),zeros(nx)];
            LMI = [LMI; H >= numAcc*eye(2*nx)];

            objective = bet;
        case 'projection-1'
            alph = sdpvar(1);
            LMI = [LMI; mapping.X-alph*eye(nx)<=-numAcc*eye(nx)];
            LMI = [LMI; mapping.Y-alph*eye(nx)<=-numAcc*eye(nx)];

            objective = alph;
        case 'projection-3'
            alph = sdpvar(1);
            H = [zeros(nx),zeros(nx,nu),mapping.Ak,mapping.Bk;...
                 zeros(nu,nx),zeros(nu),mapping.Ck,mapping.Dk;...
                 mapping.Ak',mapping.Ck',zeros(nx),zeros(nx,ny);...
                 mapping.Bk',mapping.Dk',zeros(ny,nx),zeros(ny)]+...
                alph*eye(2*nx+nu+ny);
            LMI = [LMI; H>= numAcc*eye(2*nx+nu+ny)];

            objective = alph;
        otherwise
            error('You should not be able to get here.')
    end
%% Discrete-time
else
    switch constraintType
        case 'transformation-1'
            alph = sdpvar(1);
            H = [zeros(nx),zeros(nx,nu),mapping.U,mapping.V;...
                 zeros(nu,nx),zeros(nu),mapping.W,mapping.X;...
                 mapping.U',mapping.W',zeros(nx),zeros(nx,ny);...
                 mapping.V',mapping.X',zeros(ny,nx),zeros(ny)]+...
                alph*eye(2*nx+nu+ny);
            LMI = [LMI; H>= numAcc*eye(2*nx+nu+ny)];
            LMI = [LMI; mapping.S+mapping.S'-alph*eye(nx)<=-numAcc*eye(nx)];
            LMI = [LMI; mapping.N+mapping.N'-alph*eye(nx)<=-numAcc*eye(nx)];
            LMI = [LMI; mapping.J+mapping.J'-alph*eye(nx)<=-numAcc*eye(nx)];

            objective = alph;
        case {'transformation-2','projection-2'}
            bet = sdpvar(1); assign(bet,0);
            H = [mapping.J+mapping.J',zeros(nx);
                  zeros(nx),mapping.N+mapping.N']+...
                 [zeros(nx),bet*eye(nx);
                  bet*eye(nx),zeros(nx)];
            LMI = [LMI; H >= numAcc*eye(2*nx)];

            objective = bet;
        case 'projection-1'
            alph = sdpvar(1);
            LMI = [LMI; mapping.S+mapping.S'-alph*eye(nx)<=-numAcc*eye(nx)];
            LMI = [LMI; mapping.N+mapping.N'-alph*eye(nx)<=-numAcc*eye(nx)];
            LMI = [LMI; mapping.J+mapping.J'-alph*eye(nx)<=-numAcc*eye(nx)];

            objective = alph;
        case 'projection-3'
            alph = sdpvar(1);
            H = [zeros(nx),zeros(nx,nu),mapping.Ak,mapping.Bk;...
                 zeros(nu,nx),zeros(nu),mapping.Ck,mapping.Dk;...
                 mapping.Ak',mapping.Ck',zeros(nx),zeros(nx,ny);...
                 mapping.Bk',mapping.Dk',zeros(ny,nx),zeros(ny)]+...
                alph*eye(2*nx+nu+ny);
            LMI = [LMI; H>= numAcc*eye(2*nx+nu+ny)];

            objective = alph;
        otherwise
            error('You should not be able to get here.')
    end
end

