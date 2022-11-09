function [K,gam,Xcl] = lpvlinfsyn(P,ny,nu,synopts)
%LPVLINFSYN Linf-gain suboptimal state-space affine LPV controller synthesis. 
%
%   Syntax:
%       [K, gam, Xcl] = lpvlinfsyn(P, ny, nu, synopts)
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
%   See also LPVSYN.
%

synopts.performance = 'linf';

[K,gam,Xcl] = lpvsyn(P,ny,nu,synopts);