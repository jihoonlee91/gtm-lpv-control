function lsimOpts = parselsimOpts(sys, p, u, varargin)
%PARSELSIMOPTS Parse options for LSIM of LPVREP objects
% 
%   Syntax:
%     lsimOpts = parselsimOpts(sys, varargin)
%
%   Inputs:
%       sys: LPVREP system object for which simulation is requested
%       varargin: property-value pairs
%
%   Outputs:
%       lsimOpts: struct containing the following option fields for the
%           simulation:
%
%       t: empty matrix ([]) for DT, or vector of equidistant time
%           instances
%
%       CT:
%           Solver: ODE solver specified as function handle
%           SolverOpts: ODE solver options
%       DT:
%           p0, pf: initial and final values of the scheduling signal
%           u0: initial values of the input signal (LPVIO only)
%           y0: initial values of the output signal (LPVIO only)
%

%TODO: replace errors with assert.

assert(sys.Np > 0 || sys.Nu > 0, ...
    ['Simulation of an LPV system with 0 inputs and 0 scheduling signals ', ...
    'is not currently supported']);

% Validation of input and scheduling signal
if isempty(p)
    assert(sys.Np == 0, ...
        'p cannot be empty for a system with at least 1 scheduling signal');
end
if isempty(u)
    assert(sys.Nu == 0, ...
        'u cannot be empty for a system with at least 1 input');
end
% Expand empty signals to match the number of rows of the other
if isempty(p); p = NaN(size(u, 1), sys.Np); end
if isempty(u); u = NaN(size(p, 1), sys.Nu); end
% Check rows match
if size(p, 1) ~= size(u, 1)
    error('Number of rows of p and u must be equal');
end
% Check columns match
if size(p, 2) ~= sys.Np
    error(['Number of columns of p (', ...
        int2str(size(p, 2)), ') must equal the scheduling dimension (', ...
        int2str(sys.Np), ')']);
end
if size(u, 2) ~= sys.Nu
    error(['Number of columns of u (', ...
        int2str(size(u, 2)), ') must equal the input dimension (', ...
        int2str(sys.Nu), ')']);
end

% Size of initial and final values of p
M0 = -min(min(sys.SchedulingTimeMap.Map), 0);
Mf = max(max(sys.SchedulingTimeMap.Map), 0);

parser = inputParser;

scalarOrMatOrEmpty = @(x, n, m) isscalar(x) || ...
    isempty(x) || ...
    (isnumeric(x) && all(size(x) == [n, m]));

validModes = {'fast', 'accurate'};
checkInterpMode = @(x) any(validatestring(x,validModes));
validModesIODT = {'fast', 'large'};
possibleSolversIODT = @(x) any(validatestring(x,validModesIODT));

% Time vector
parser.addOptional('t', []);

% Noise
if isa(sys, 'lpvidpoly')
    parser.addOptional('e', [], ...
        @(x) isempty(x) || (isnumeric(x) && size(x, 2) == sys.Ny));
end

% Initial state
if isa(sys, 'lpvlfr') || isa(sys, 'lpvidss')
    parser.addOptional('x0', zeros(sys.Nx, 1), ...
        @(x) isnumeric(x) && isvector(x) && ...
            (numel(x) == sys.Nx || numel(x) == 1));
end

if isa(sys, 'lpvidss')
    parser.addOptional('w', [], ...
        @(x) isempty(x) || isnumeric(x));
end

if isdt(sys)
    parser.addParameter('p0', zeros(M0, sys.Np), ...
        @(x) scalarOrMatOrEmpty(x, M0, sys.Np));
    parser.addParameter('pf', zeros(Mf, sys.Np), ...
        @(x) scalarOrMatOrEmpty(x, Mf, sys.Np));
    if isa(sys, 'lpvio') || isa(sys, 'lpvidpoly')
        parser.addParameter('u0', zeros(sys.Nb, sys.Nu), ...
            @(x) scalarOrMatOrEmpty(x, sys.Nb, sys.Nu));
        parser.addParameter('y0', zeros(sys.Na, sys.Ny), ...
            @(x) scalarOrMatOrEmpty(x, sys.Na, sys.Ny));
        parser.addParameter('solverMode', 'fast', ...
            @(x) possibleSolversIODT(x));
    end
    if isa(sys, 'lpvidpoly')
        parser.addParameter('e0', zeros(sys.Nc, sys.Ny), ...
            @(x) scalarOrMatOrEmpty(x, sys.Nc, sys.Ny));
    end
end

% ODE solver and options
if isct(sys)
    parser.addParameter('Solver', @ode45, ...
        @(x) isa(x, 'function_handle') && ...
            isequal(x, @ode45) || isequal(x, @ode23) || ...
            isequal(x, @ode113) || isequal(x, @ode15s));
    parser.addParameter('SolverOpts', odeset, ...
        @(x) isa(x, 'struct'));
    parser.addParameter('SolverInterpolationMode',...
        'fast', checkInterpMode)
end

% Parse PV pairs
parser.parse(varargin{:});

%% Validate and assign to lsimOpts
t = parser.Results.t;
if ~isempty(t)
    % Check if t is equidistant
    assert(isvector(t) && ...
        isnumeric(t) && isreal(t) && ...
        norm(diff(t) - mean(diff(t))) / numel(t) < 10 * eps, ...
        't must be a vector of equidistant time instances');
    % Check number of samples
    assert(numel(t) == size(u, 1), ...
        'Number of time samples specified in t, u and p does not match');
    % Check increasing
    assert(all(diff(t) > 0), ...
        't must be an increasing vector');
end
t = t(:);

% DT options
if isdt(sys)
    % Check if t is compatible with sampling time Ts
    if ~isempty(t)
        if sys.Ts ~= -1
            assert(norm(diff(t) - sys.Ts) / numel(t) < 10 * eps, ...
                't must be a vector of equidistance time instances corresponding to the sampling time Ts');
        end
    end
    lsimOpts.t = t;
    p0 = parser.Results.p0;
    pf = parser.Results.pf;
    % convert empty inputs to scalars
    if isempty(p0); p0 = 0; end
    if isempty(pf); pf = 0; end
    % convert scalar inputs to matrix
    if isscalar(p0)
        p0 = ones(M0, sys.Np) * p0;
    end
    if isscalar(pf)
        pf = ones(Mf, sys.Np) * pf;
    end
    lsimOpts.p0 = p0; lsimOpts.pf = pf;
    if isa(sys, 'lpvio') || isa(sys, 'lpvidpoly')
        u0 = parser.Results.u0;
        y0 = parser.Results.y0;
        if isempty(y0); y0 = 0; end
        if isempty(u0); u0 = 0; end
        if isscalar(y0)
            y0 = ones(sys.Na, sys.Ny) * y0;
        end
        if isscalar(u0)
            u0 = ones(sys.Nb, sys.Nu) * u0;
        end
        lsimOpts.y0 = y0; lsimOpts.u0 = u0;
        lsimOpts.solverMode = parser.Results.solverMode;
    end
    if isa(sys, 'lpvidpoly')
        e0 = parser.Results.e0;
        e = parser.Results.e;
        % Generate noise
        [R, cholP] = chol(sys.NoiseVariance);
        isPosDef = cholP == 0;
        if ~isPosDef
            R = zeros(sys.Ny);
        end
        if isempty(e)
            if isempty(e0)
                e0 = randn(sys.Nc, sys.Ny) * R;
            end
            e = randn(size(p, 1), sys.Ny) * R;
        end
        if isempty(e0)
            e0 = randn(sys.Nc, sys.Ny) * R;
        end
        lsimOpts.e0 = e0;
        lsimOpts.e = e;
    end
    if isa(sys, 'lpvidss')
        w = parser.Results.w;
        % Generate noise
        [R, cholP] = chol(sys.NoiseVariance);
        isPosDef = cholP == 0;
        if ~isPosDef
            R = zeros(sys.Nu);
        end
        if isempty(w)
            v = randn(size(p, 1), sys.Nu) * R;
            if strcmp(typeNoiseModel(sys), 'innovation')
                e = v;
            else
                e = randn(size(p, 1), sys.Nu) * R;
            end
        else
            % Split w
            if strcmp(typeNoiseModel(sys), 'innovation')
                v = w;
                e = w;
            else
                v = w(1:sys.Nu);
                e = w(sys.Nu+1:end);
            end
        end
        lsimOpts.v = v;
        lsimOpts.e = e;
    end
end

% CT options
if isct(sys)
    assert(~isempty(t), 't must not be empty');
    lsimOpts.t = t;
    lsimOpts.Solver = parser.Results.Solver;
    lsimOpts.SolverOpts = parser.Results.SolverOpts;
    
    lsimOpts.SolverMode = parser.Results.SolverInterpolationMode;
end

% State-space options
if isa(sys, 'lpvlfr') || isa(sys, 'lpvidss')
    x0 = parser.Results.x0;
    % Expand scalar
    if isscalar(x0)
        x0 = x0 * ones(sys.Nx, 1);
    end
    % Make column vector
    x0 = x0(:);
    lsimOpts.x0 = x0;
end

end

