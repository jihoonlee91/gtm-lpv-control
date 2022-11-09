function M = hambo(v, ne, Ts)
%HAMBO Generate discrete-time Hambo basis
%
% Syntax:
%   M = hambo(v, ne)
%   M = hambo(v, ne, Ts)
%
% Inputs:
%   v: vector of (stable) pole locations of the generating inner function.
%   ne: number of extensions of the inner function.
%   Ts: sample time (-1 if unspecified).
%
% Outputs:
%   M: minimal state-space realization of Hambo basis
%
% See also: INNERFUNCTION
%

%% Input validation
narginchk(2, 3);
nargoutchk(0, 1);
if ~((isempty(v) || isvector(v)) && isnumeric(v))
    error('''v'' must be a numeric vector');
end
if ~((all(abs(v) < 1)))
    error('All poles in ''v'' must be stable (abs(v) < 1)');
end
if ~((isint(ne)) && ne > 0)
    error('''ne'' must be a positive integer');
end
if nargin <= 2
    Ts = -1;
end
if ~(isscalar(Ts) && isnumeric(Ts) && Ts ~= 0 && isreal(Ts))
    error('''Ts'' must be a positive real scalar or -1');
end

%% Inner function
[a, b, c, d] = obfpl2ss(v);

%% Extend inner function
[ae,be,ce,de] = obfextend(a, b, c, d, ne);
M = ss(ae, be, ce, de, Ts);

end

