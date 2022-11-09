function globalsys = lpvinterpss(lpvsys, interpMethod, interpOpts)
%LPVINTERPSS Convert LPVGRIDSS object to global LPV model using
%interpolation or approximation
%
% Syntax:
%   globalsys = lpvinterpss(sys, interpMethod, interpOpts)
%
% Inputs:
%   sys: LPVGRIDSS object from which the linearized models and information
%       on the scheduling signals is extracted.
%   interpMethod: interpolation method, specified as a character
%       vector. The following interpolation and approximation methods are supported:
%           'pwlinear': piece-wise linear
%           'pwpoly': piece-wise polynomial. The exact method is specified
%               through the interpOpts argument.
%           'poly': global polynomial approximation.
%           'behavioral': behavioral approximation. Note: this is an
%               approximation method, i.e., the LPV representation may not
%               be exactly equal to the linear systems passed as input at
%               the grid points.
%   interpOpts: options for the interpolation or approximation. The
%       type depends on the selected method:
%           'pwlinear': no options are available.
%           'pwpoly': a character vector indicating the interpolation
%               method. Options available:
%                   'linear': same as specifying 'pwlinear' as
%                       interpolation method.
%                   'spline': (default) cubic-spline based
%                       interpolation. Uses INTERPN.
%                   'makima' / 'cubic' / 'pchip': these methods match
%                   the methods in the built-in MATLAB function
%                   INTERPN. See DOC INTERPN for more information. 
%           'poly': a structure with for each scheduling signal, a
%               field with the same name as the scheduling signal. The
%               corresponding value is the degree of the polynomial that
%               will be used. Note that this implies that cross terms are
%               currently not supported! Example:
%                   struct('p', 2, 'q', 1) --> quadratic polynomial for
%                   'p' and affine polynomial for 'q'.
%               The degree must be at least 1 and at most the number of
%               grid points minus 1.
%           'behavioral': a structure with the following fields:
%               'init_sys': an LPV-SS representation from which the
%                   structure of the Delta block and the initial values for
%                   the model parameters are extracted and used to
%                   determine the approximate global system.
%               'hinfstructOptions': options structure as used by
%                   HINFSTRUCT. HINFSTRUCT is internally called to
%                   calculate the approximate global system through
%                   H_{infty} behavioral approximation.
% Outputs:
%   globalsys: LPVREP object containing global model representation.
%
% Limitations:
%   * Currently only static dependence is supported (i.e., there can be
%   no dynamic dependence in the timemap object).
%   * Empty input or output channels are not supported.
%   * Each scheduling signal must have at least two grid points.
%
% See also LPVGRIDSS
%

%% Input validation
narginchk(1, 3);
% Check sys and extract variables for use during interpolation
assert(isa(lpvsys, 'lpvgridss'), '''sys'' must be an lpvgridss object');
grid = lpvsys.SamplingGrid;
sys = lpvsys.ModelArray;
f = fieldnames(grid);
if isct(sys)
    domain = 'ct';
else
    domain = 'dt';
end
tm = timemap(0, domain, 'Name', f);
sz = size(lpvsys);
N = prod(sz(3:end));
nx = lpvsys.Nx;
ny = lpvsys.Ny;
nu = lpvsys.Nu;
nrho = lpvsys.Np;
Ts = lpvsys.Ts;
% gridCell is required for PBINTERP
gridCell = cell(1, numel(f));
for i=1:numel(f)
    gridCell{i} = grid.(f{i});
end

V = zeros(size(grid.(f{1}),2), 1);
for i=2:numel(f)
    V = V .* zeros([ones(1, i-1) size(grid.(f{i}),2)]);
end
assert(numel(V) == N, 'LPVcore:gridds:ConstructorArgin',...
        'number of systems doesn''t fit the grid.')
% Interpolation method
if nargin < 2
    interpMethod = 'pwlinear';
end
assert(ischar(interpMethod), '''interpMethod'' must be a character vector');
assert(any(strcmp(interpMethod, {'pwlinear', 'pwpoly', 'poly', 'behavioral'})), ...
    ['Unsupported interpolation method ', interpMethod]);
% Options set for interpolation method
if nargin < 3
    if any(strcmp(interpMethod, {'poly', 'behavioral'}))
        error('Interpolation options must be specified for interpolation method ''%s''', interpMethod);
    end
    if strcmp(interpMethod, 'pwpoly')
        interpMethod = 'spline';
    end
else
    if any(strcmp(interpMethod, {'pwlinear'}))
        error('''interpOpts'' is not supported for selected interpolation method ''%s''.', interpMethod);
    end
end
if strcmp(interpMethod, 'poly')
    % Check interpOpts for 'poly' interpolation
    assert(isstruct(interpOpts), ...
        '''interpOpts'' must be a struct for interpMethod ''poly''');
    assert(numel(fieldnames(interpOpts)) == numel(sz) - 2, ...
        'Number of fields of ''interpOpts'' (%i) must match grid dimension (%i)', ...
        numel(fieldnames(interpOpts)), numel(sz) - 2);
    fi = fieldnames(interpOpts);
    assert(isequal(f, fi), '''grid'' and ''interpOpts'' must have identical field names (and same ordering)');
    % Ensure degrees are valid
    for i=1:numel(fi)
        assert(interpOpts.(fi{i}) >= 1 && interpOpts.(fi{i}) <= sz(i+2) - 1, ...
            'Polynomial degree for scheduling signal ''%s'' is %i but must be positive and not larger than %i', ...
            fi{i}, interpOpts.(fi{i}), sz(i+2) - 1);
    end
end
if strcmp(interpMethod, 'behavioral')
    % Check interpOpts for 'behavioral' approximation
    assert(isstruct(interpOpts) && ...
        any(strcmp('init_sys', fieldnames(interpOpts))), ...
        '''interpOpts'' must be a struct with fields ''init_sys'' and optionally ''hinfstructOptions''');
    if ~any(strcmp('hinfstructOptions', fieldnames(interpOpts)))
        % Use default options set to HINFSTRUCT
        interpOpts.hinfstructOptions = hinfstructOptions;
    end
    assert(isa(interpOpts.init_sys, 'lpvss'), ...
        '''interpOpts.init_sys'' must be an ''lpvss'' object.');
    assert(hasStaticSchedulingDependence(interpOpts.init_sys), 'Initial LPV-SS model must have static dependence on scheduling signal');
    assert(interpOpts.init_sys.Nrho == numel(sz) - 2, ...
        'Initial LPV-SS model must have same number of scheduling signals as grid dimension');
    assert(isa(interpOpts.hinfstructOptions, 'rctoptions.hinfstruct'), ...
        '''interpOpts.hinfstructOptions'' must be an ''hinfstructOptions'' object for behavioral approximation.');
end
if strcmp(interpMethod, 'pwpoly')
    assert(ischar(interpOpts) && ...
        any(strcmp(interpOpts, {'pchip', 'cubic', 'makima', 'spline', 'linear'})), ...
        '''interpOpts'' specified for ''pwpoly'' not supported. Choose from: ''pchip'', ''cubic'', ''makima'', ''spline'', ''linear''');
end

%% Construct lpvgridss object
% Piece-wise linear interpolation
if any(strcmp(interpMethod, {'pwlinear', 'pwpoly'}))
    if strcmp(interpMethod, 'pwlinear')
        interpOpts = 'linear';
    end
    % Number of grid points
    A = NaN(nx, nx, N);
    B = NaN(nx, nu, N);
    C = NaN(ny, nx, N);
    D = NaN(ny, nu, N);
    bfuncs = cell(1, N);
    for i=1:N
        Vt = V;
        Vt(i) = 1;
        A(:,:,i) = sys(:, :, i).A;
        B(:,:,i) = sys(:, :, i).B;
        C(:,:,i) = sys(:, :, i).C;
        D(:,:,i) = sys(:, :, i).D;
        bfuncs{i} = pbinterp(gridCell,Vt, interpOpts);
    end
    A = pmatrix(A, bfuncs, 'SchedulingTimeMap', tm);
    B = pmatrix(B, bfuncs, 'SchedulingTimeMap', tm);
    C = pmatrix(C, bfuncs, 'SchedulingTimeMap', tm);
    D = pmatrix(D, bfuncs, 'SchedulingTimeMap', tm);
elseif strcmp(interpMethod, 'poly')
    % Global polynomial approximation
    %
    % Assuming the scheduling signals are named rho1, ...,
    % rhoN, the basis functions will be of the following form:
    %
    %   { 1, rho1, rho1^2, rho1^(d1), rho2, ..., rhoN^(dN-1), rhoN^(dN)}
    %
    % These basis functions are first evaluated for each grid
    % point and placed into a matrix Rho with each column
    % corresponding to a grid point.
    %
    % To find the coefficients of the pmatrix objects
    % representing the system parameters, a least squares
    % problem of the form P * Rho = C is solved. Each row of P
    % and C corresponds to a vectorized entry of the
    % state-space matrices [A, B; C, D]. Each column of C
    % corresponds to a point in the sampling grid.
    %
    % Step 1: construct Rho
    % Field names of interpOpts are stored in 'fi'
    d = NaN(1, numel(fi));  % vector of polynomial degrees
    for i=1:numel(fi)
        d(i) = interpOpts.(fi{i});
    end
    % Number of basis functions
    % Note: d includes the constant term for each scheduling
    % signal, i.e., d = 0. This term is shared across all
    % scheduling signals.
    nbasis = 1 + sum(d);
    bfuncs = cell(1, nbasis);
    % N: number of grid points
    Rho = NaN(nbasis, N);
    % Constant term
    Rho(1, :) = 1;
    bfuncs{1} = pbconst;
    bfuncsidx = 2;
    for i=1:nrho
        for j=1:d(i)
            if j == 1
                bfuncs{bfuncsidx} = pbaffine(i);
            else
                degrees = zeros(1, i);
                degrees(end) = j;
                bfuncs{bfuncsidx} = pbpoly(degrees);
            end
            bfuncsidx = bfuncsidx + 1;
        end
    end
    C = NaN((nx + ny) * (nx + nu), N);
    indx = cell(1, nrho);
    % Loop over grid points
    for i=1:N
        % Vector of scheduling signals at this grid point
        [indx{:}] = ind2sub(sz(3:end), i);
        % Loop over scheduling signals and keep track of which
        % rows of Rho using an index counter (start at 2 since
        % constant term is already populated).
        rhorow = 2;
        for j=1:nrho
            g = grid.(f{j})(indx{j});
            Rho(rhorow:rhorow+d(j)-1, i) = cumprod(g * ones(d(j), 1));
            rhorow = rhorow + d(j);
        end
        % Populate C from sys
        s = sys(:, :, i);
        smat = [s.A, s.B; s.C, s.D];
        C(:, i) = smat(:);
    end
    % Solve least squares fit P * Rho = C for P
    P = C / Rho;
    % Resize for pmatrix
    P = reshape(P, [nx + ny, nx + nu, nbasis]);
    tm = timemap(0, 'ct', 'Name', fi);
    A = pmatrix(P(1:nx, 1:nx, :),           bfuncs, 'SchedulingTimeMap', tm);
    B = pmatrix(P(1:nx, nx+1:end, :),       bfuncs, 'SchedulingTimeMap', tm);
    C = pmatrix(P(nx+1:end, 1:nx, :),       bfuncs, 'SchedulingTimeMap', tm);
    D = pmatrix(P(nx+1:end, nx+1:end, :),   bfuncs, 'SchedulingTimeMap', tm);
elseif strcmp(interpMethod, 'behavioral')
    % HINFID requires local LTI systems and grid points in cell
    % arrays (since irregular / sparse grids are supported).
    sysCell = cell(1, N);
    gridCell = cell(1, N);
    indx = cell(1, nrho);
    for i=1:N
        sysCell{i} = sys(:, :, i);
        [indx{:}] = ind2sub(sz(3:end), i);
        gridCelli = NaN(1, nrho);
        for j=1:nrho
            gridCelli(j) = grid.(f{j})(indx{j});
        end
        gridCell{i} = gridCelli;
    end
    hinfidss = lpvss(hinfid(sysCell, gridCell, ...
        interpOpts.init_sys, interpOpts.hinfstructOptions));
    A = hinfidss.A;
    B = hinfidss.B;
    C = hinfidss.C;
    D = hinfidss.D;
end
globalsys = lpvss(A,B,C,D,Ts, ...
        'InputName', sys.InputName, ...
        'OutputName', sys.OutputName, ...
        'InputUnit', sys.InputUnit, ...
        'OutputUnit', sys.OutputUnit, ...
        'InputGroup', sys.InputGroup, ...
        'OutputGroup', sys.OutputGroup, ...
        'InputDelay', sys.InputDelay);

end

