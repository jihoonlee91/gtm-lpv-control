function tm = randtimemap(tmin, tmax, n, domain, varargin)
%RANDTIMEMAP Returns randomly generated timemap.
%   tm = RANDTIMEMAP(tmin, tmax, n, domain)
%   tm = RANDTIMEMAP(tmin, tmax, n, domain, 'SchedulingName', {'p1', 'p2'})
%
%   Syntax: 
%       tm = randtimemap(tmin, tmax, n, domain, varargin)
%
%   Inputs:
%       tmin (int): minimum time-shift (DT) or derivative order (CT).
%       tmax (int): maximum time-shift (DT) or derivative order (CT).
%       n (int): number of elements in the randomly generated time-map.
%       domain ('ct' or 'dt'): time domain. Either continuous-time ('ct')
%           or discrete-time ('dt').
%       varargin: these input arguments shadow the remaining input 
%           arguments of timemap.
%
%   Outputs:
%       tm (timemap): randomly-generated time-map.
%
%   Example:
%       tm = timemap(-2, 2, 3, 'dt')
%       tm = timemap(0, 10, 5, 'ct', 'SchedulingName', {'p1', 'p2'})
%
%   See also: TIMEMAP.

if tmin < 0 && strcmp(domain, 'ct')
    LPVcore.error('LPVcore:timemap:tMCTWithNegMap');
end

t = randsample(linspace(tmin, tmax, tmax - tmin + 1), n)';
tm = timemap(t, domain, varargin{:});

end

