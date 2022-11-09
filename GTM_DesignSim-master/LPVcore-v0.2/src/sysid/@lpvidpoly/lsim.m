function [y, t, e, yp, v] = lsim(sys, p, u, varargin)
%LSIM Simulating the lpvidpoly object
%
%   Will simulate the following LPVIDPOLY model
%
%   A(q,p(t))  y(t)  = [F(q,p(t)) \ B(q,p(t))] u(t) + [C(q,p(t)) \ D(q,p(t))] u(t)
%
%   where each polynomial is a summation in the pulse basis q^{-1}, e.g.,
%
%   A(q,p(t)) = sum( A{i} q^-i , i=1,..,na),
%
%   with A{i} a pmatrix, real matrix, or []. It is assumed that
%   A(.), C(.), D(.), and F(.) are monic polynomials. Type "help lpvidpoly"
%   for more information.
%
%   Syntax:
%   lsim(sys,p)
%   lsim(sys,p,u)
%   lsim(sys,p,u,t)
%   lsim(sys,p,u,t,e)
%   lsim(sys,p,u,t,e,'Property1',Value1,...,'PropertyN',ValueN)
%   y = lsim(___)
%   [y,t]        = lsim(___)
%   [y,t,e]      = lsim(___)
%   [y,t,e,yp]   = lsim(___)
%   [y,t,e,yp,v] = lsim(___)
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
%   v and yp are not computed in case sys.A is not empty.
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
%       'y0': nc-by-ny matrix, scalar or []. Default: 0
%       'u0': nb-by-nu matrix, scalar or []. Default: 0
%       'e0': nc-by-ny matrix, scalar or []. Default: 0
%

% TODO: 
% - Implement simulation of CT systems.
assert(isdt(sys), ...
    'LSIM is only supported for DT LPVIDPOLY objects');

lsimOpts = LPVcore.parselsimOpts(sys, p, u, varargin{:});
t = lsimOpts.t;
p0 = lsimOpts.p0;
pf = lsimOpts.pf;
u0 = lsimOpts.u0;
y0 = lsimOpts.y0;
e = lsimOpts.e;
e0 = lsimOpts.e0;

%% Algorithm
% The simulation of the following system:
%
%   A y = G u + H e = F \ B u + D \ C e
%
% ...is solved by calculating yd, ys:
%
%   A y = yp + v
%
% for which it holds that F yd = B ys, D v = C e
%
if sys.Nu > 0
    G = lpvio(sys.F.Mat, sys.B.Mat, sys.Ts);
    G.InputDelay = sys.InputDelay;
    yp = lsim(G, p, u, t, 'p0', p0, 'pf', pf, 'u0', u0, 'y0', zeros(sys.Nf, sys.Ny));
else
    yp = zeros(size(p, 1), sys.Ny);
end
H = lpvio(sys.D.Mat, sys.C.Mat, sys.Ts);

% TODO: yd0, ys0
v = lsim(H, p, e, t, 'p0', p0, 'pf', pf, 'u0', e0, 'y0', zeros(sys.Nd, sys.Ny));

S = lpvio(sys.A.Mat, {eye(sys.Ny)}, sys.Ts);
[y, t] = lsim(S, p, yp + v, t, 'p0', p0, 'pf', pf, 'y0', y0);
% Update deterministic output of system
[yp, t] = lsim(S, p, yp, t, 'p0', p0, 'pf', pf, 'y0', y0);

if nargout == 0
    figure;
    plotlsim(sys, y, p, u, t);
end
    
end
