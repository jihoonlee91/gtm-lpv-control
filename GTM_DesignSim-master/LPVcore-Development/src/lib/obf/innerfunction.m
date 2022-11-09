function Gb = innerfunction(v, Ts)
%INNERFUNCTION Construct discrete-time inner function (or Blaschke product)
%
% Syntax:
%   Gb = innerfunction(v)
%   Gb = innerfunction(v, Ts)
%
% Inputs:
%   v: vector of pole locations
%   Ts: sample time (set to -1 if unspecified)
%
% Outputs:
%   Gb: a minimal state-space realization of inner function corresponding to pole
%       locations. Sign is taken to be positive (+).
%

%% Input validation
narginchk(1, 2);
nargoutchk(0, 1);
if ~((isempty(v) || isvector(v)) && isnumeric(v))
    error('''v'' must be a numeric vector');
end
if ~((all(abs(v) < 1)))
    error('All poles in ''v'' must be stable (abs(v) < 1)');
end
if nargin <= 1
    Ts = -1;
end
if ~(isscalar(Ts) && isnumeric(Ts) && Ts ~= 0 && isreal(Ts))
    error('''Ts'' must be a positive real scalar or -1');
end

% Reference: R. Toth, "Modeling and identification of linear parameter-varying
% systems" (2010)
z = tf([1, 0], 1, Ts);

% Eq. (2.10) on page 23
Gb = 1;
for i=1:numel(v)
    Gb = Gb * (1 - conj(v(i)) * z) / (z - v(i));
end

Gb = ss(Gb);

end

