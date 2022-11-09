function [K,gamma,Xcl] = lpvsyn(P,ny,nu,synopts)
%LPVSYN Suboptimal state-space affine LPV controller synthesis. 
%
%   Syntax:
%       [K, gamma, Xcl] = lpvsyn(P, ny, nu, ...)
%
%       [K, gamma, Xcl] = lpvsyn(P, ny, nu, synopts)
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
%       gamma: Upperbound for the gain of the closed-loop system.
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
%   Ali, M., and Werner, H., "Discrete-Time LPV Controller Synthesis Using 
%   Dilated LMIs with Application to an Arm-Driven Inverted Pendulum," 
%   Proc. of 18th World Congress IFAC, (2011).
%
%   Scherer, C. W. and Weiland, S., "Linear Matrix Inequalities in Control,
%   " Jan-2015. <a href="https://www.imng.uni-stuttgart.de/mst/files/LectureNotes.pdf">[Online]</a>.  
%

%% Input parser
% Check if synthesis options are given
if ~exist('synopts','var')
    synopts = lpvsynOptions;
end

%% Variables
perfType = synopts.performance;

%% Checks
assert(isa(P,'lpvss'),'System is not an lpvss object.');
assert(checkNyNu(ny,nu),'Inputs ny and/or nu are not integer valued or not positive.');
assert(all([isaffine(P.A),isaffine(P.B),isaffine(P.C),isaffine(P.D)]),...
       'System is not affinely dependent on the parameter.');

%% Check which type of controller we need to synthesize.
switch perfType
    case 'passive'
        [K,Xcl]       = lpvpassyn(P,ny,nu,synopts);
        gamma = []; % dummy gamma
    case 'l1'
        [K,gamma,Xcl] = lpvl1syn(P,ny,nu,synopts);
    case 'h2'
        [K,gamma,Xcl] = lpvh2syn(P,ny,nu,synopts);
    case 'l2'
        [K,gamma,Xcl] = lpvl2syn(P,ny,nu,synopts);
    otherwise
        warning(['The controller gain based on %s is unknown.\n',...
            'Currently supported controller types are l2, h2, l1 and passive.\n',...
            'Generating a controller based on L2-gain.'],type);
        [K,gamma,Xcl] =  lpvl2syn(P,ny,nu,synopts);
end
end

function out = checkNyNu(ny,nu)
% check if it is a whole number
check1 = (round(ny,0) == ny) & (round(nu,0) == nu); 
% check if they are positive
check2 = (ny > 0) & (nu > 0); 
out = check1 & check2;
end