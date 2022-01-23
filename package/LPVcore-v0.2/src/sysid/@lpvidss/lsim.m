function [y, t, w, yp] = lsim(sys, p, u, varargin)
%LSIM Simulating the lpvidss object
%
%   Type "help lpvidss"
%   for more information on lpvidss models.
%
%   Syntax:
%   lsim(sys,p)
%   lsim(sys,p,u)
%   lsim(sys,p,u,t)
%   lsim(sys,p,u,t,x0)
%   lsim(sys,p,u,t,x0,w)
%   lsim(sys,p,u,t,x0,w,'Property1',Value1,...,'PropertyN',ValueN)
%   y = lsim(___)
%   [y,t]        = lsim(___)
%   [y,t,w]        = lsim(___)
%   [y,t,w,yp]     = lsim(___)
%
%   lsim simulates the (time) response of continuous or discrete linear
%   parameter-varying systems to arbitrary inputs. When invoked without
%   left-hand arguments, lsim plots the response on the screen.
%
%   The input u is an array having as many rows as time samples (length(t))
%   and as many columns as system inputs.
%
%   The scheduling signal p is an array having as many rows as time samples
%   (length(t)) and as many columns as system scheduling dimension.
%
%   x0 is the initial state and set to 0 if left empty.
%
%   yp is the deterministic output, i.e., the output when not considering
%   noise.
%
%   The input w = e for innovation form models and w = [v, e] for general
%   form models. Both e and v are matrices having as many rows as time
%   samples (length(t)) and as many columns as the input dimension.
%
%   
%
%   Property/value pairs
%
%       'p0': M0-by-np matrix, scalar or []. The M0 initial values
%           of p. M0 is the maximum negative time-shift (DT) or the
%           maximum derivative order (CT) of the time-map.
%           Default: 0
%       'pf': Mf-by-np matrix, scalar or []. The Mf final values of
%           p. Mf is the maximum positive time-shift (DT) or the
%           maximum derivative order (CT) of the time-map.
%           Default: 0
%

assert(isdt(sys), ...
    'LSIM is only supported for DT LPVIDSS objects');

lsimOpts = LPVcore.parselsimOpts(sys, p, u, varargin{:});
t = lsimOpts.t;
x0 = lsimOpts.x0;

%% Algorithm
sys_equiv = equivlpvss(sys);
w = [lsimOpts.v, lsimOpts.e];
% TODO: add options here
[y, t] = lsim(sys_equiv, p, [u, w], t, x0);
% Calculate yp if requested
[yp, t] = lsim(sys_equiv, p, [u, zeros(size(w))], t, x0);

if nargout == 0
    figure;
    plotlsim(sys, y, p, u, t);
end

end
