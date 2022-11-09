classdef pmatrix
    %PMATRIX Parameter-varying matrix
    %
    % Represents a parameter-varying matrix which is an affine combination
    % of basis functions that depend on the extended scheduling signal rho:
    %
    %   P = P0 * phi0(rho) + ... + Pn * phin(rho)
    %
    % with P0, ..., Pn constant matrices and phi0, ..., phin basis
    % functions mapping the extended scheduling vector rho to a scalar.
    % PMATRIX objects can be created in several ways. First, by calling
    % PMATRIX directly. Second, by using functions such as
    % PREAL that return a preset PMATRIX object. Third, by combining
    % PMATRIX objects through multiplication, addition or division. Here,
    % the syntax for directly calling PMATRIX is explained. 
    % 
    %   Syntax 1:
    %       P = pmatrix(mat)
    %
    %   If no scheduling timemap is specified, then a one-dimensional scheduling vector named 
    %   'p' is automatically created (in continuous-time). Note that, in this case we have p = rho. 
    %   If mat is a matrix, then the resulting pmatrix will represent P = mat * p. mat can also
    %   be a 3-dimensional double with the size of the third dimension
    %   being at most 2, in which case an affine dependence is created.
    %   Example:
    %
    %       >> P = pmatrix(cat(3, 2, 4))
    %       P = 2 * p + 4
    %
    %   Syntax 2:
    %       P = pmatrix(mat, bfuncs, 'SchedulingTimeMap', timemap)
    %       P = pmatrix(mat, bfuncs, Name, Value)
    %
    %   This syntax provides the most direct way of constructing a pmatrix
    %   object, by passing a cell array of basis functions, bfuncs, and a
    %   scheduling timemap mapping the scheduling vector p to the extended
    %   scheduling vector rho. Instead of SchedulingTimeMap, the following
    %   Name, Value pairs can be specified to construct a timemap:
    %
    %       SchedulingDimension: dimension of scheduling signal.
    %       SchedulingMap: vector of time shifts (DT) and derivatives (CT)
    %           from which the extended scheduling vector is formed.
    %       SchedulingDomain: time-domain. Possible values: 'ct' or 'dt'.
    %       SchedulingName: cell-array of signal names of scheduling signal.
    %       SchedulingUnit: cell-array of signal units of scheduling
    %           signal.
    %       SchedulingRange: cell-array of vectors with 2 elements
    %           indicating the min. and max. value a scheduling signal can
    %           assume.
    %       SchedulingRateBound: cell-array of vectors with 2 elements
    %           indicating the min. and max. rate of change a scheduling
    %           signal can assume.
    %
    %   For more information on these parameters, see TIMEMAP.
    %
    %   Note that the number of basis functions should match
    %   the size of the third dimension of mat. Example:
    %
    %       >> tm = timemap(0, 'ct', 'SchedulingDimension', 2)
    %       >> P = pmatrix(cat(3, 1, 2, 3), {pbconst, pbpoly(1),
    %       pbpoly(2)}, 'SchedulingTimeMap', tm)
    %
    %   Syntax 3:
    %       P = pmatrix(mat, Name, Value)
    %
    %   This syntax can be used to automatically create basis functions in
    %   several scenarios. For example, to create a linear PMATRIX consisting of
    %   arbitrarily many scheduling signals, use the following:
    %
    %       >> nrho = 3
    %       >> P = pmatrix(ones(1, 1, nrho), ...
    %               'SchedulingDimension', 3)
    %       P = p1 + p2 + p3
    %
    %   An affine term is added automatically if the dimension of mat
    %   exceeds nrho by 1, e.g.:
    %
    %       >> nrho = 3
    %       >> P = pmatrix(ones(1, 1, nrho + 1), ...
    %               'SchedulingDimension', 3)
    %       P = p1 + p2 + p3 + 1
    %
    %   Besides linear and affine dependence, polynomial
    %   dependence is also possible, by specifying BasisType:
    %
    %       >> P = pmatrix(ones(1, 1, 3), 'BasisType', 'poly')
    %       P = 1 + p + p^2
    %
    %   If the dimension of rho is more than 1, or if you want control of
    %   the degrees of the monomial terms, specify BasisParametrization:
    %
    %       >> P = pmatrix(ones(1, 1, 3), 'BasisType', 'poly', ...
    %           'BasisParametrization', {2, 3, 4})
    %       P = p^2 + p^3 + p^4
    %
    %       >> P = pmatrix(ones(1, 1, 3), 'BasisType', 'poly', ...
    %           'BasisParametrization', {[1, 0], [0, 1], [1, 1]}, ...
    %           'SchedulingDimension', 2)
    %       P = p1 + p2 + p1 * p2
    %
    %   The following parameters are accepted:
    %       BasisType: 'affine' (default), 'poly' or 'custom'.
    %       BasisParametrization: cell-array of basis function
    %       parametrization. The parametrization depends on the basis type:
    %           'affine': positive integer corrsponding to the index of the ext.
    %               scheduling vector rho, or 0 for a constant term.
    %           'poly': vector with no more elements than the number of
    %               ext. scheduling signals. Each element of this vector is the
    %           degree to which the ext. scheduling signal is raised.
    %               'custom': function handle. See PBCUSTOM.
    %
    % See also PBASIS, PBCONST, PBAFFINE, PBPOLY, PBCUSTOM, TIMEMAP
    
    properties (SetAccess='protected')
        % Coefficients specified as a 3-dimensional double array.
        % The first two dimensions are along the rows and columns of the
        % parameter varying matrix, and the third dimension is along the basis functions.
        matrices
        % Cell array of basis functions (objects derived from pbasis). The
        % number of basis functions should correspond to the third
        % dimension of the matrices property.
        %
        % See also PBASIS
        bfuncs = {}
    end
    
    properties (Dependent)
        % The scheduling timemap object mapping the scheduling signal to
        % the extended scheduling signal.
        %
        % See also TIMEMAP
        timemap
        % The dimension of the scheduling signal
        %
        % See also TIMEMAP
        Np
    end
    
    properties (Hidden, Access='private')
        numbfuncs   % (int) number of bfuncs
        timemap_    % (timemap)
    end
    
    methods(Hidden, Static)
        varargout = commonrho_(varargin)
    end
    
    methods
        function obj = pmatrix(varargin)
            %PMATRIX Construct an instance of this class
            %
            %   Syntax:
            %       P = pmatrix(mat, bfuncs, Name, Value)
            %       P = pmatrix(mat, Name, Value)
            %
            %   Inputs:
            %       mat: 3D tensor specifying the matrices.
            %       bfuncs: cell array of pbasis objects.
            %
            %   Name, Value pairs:
            %       'SchedulingTimeMap': timemap object.
            %       'BasisType': 'affine', 'poly' or 'custom'.
            %       'BasisParametrization': the interpretation and format
            %       of this argument depends on the basis type:
            %           'affine': cell array of indexes.
            %           'poly': cell arrray of degrees.
            %           'custom': cell array of function handles.
            %           'interp': cell array of interpolation functions
            %
            if nargin == 1 && isa(varargin{1}, 'pmatrix')
                obj = varargin{1};
                return;
            end
            p = inputParser;
            isValidMat = @(x) isnumeric(x) && ...
                ndims(x) >= 2 && ...
                ndims(x) <= 3;
            % Constant matrices
            addRequired(p, 'mat', isValidMat);
            addOptional(p, 'bfuncs', {}, ...
                @(x) iscellof(x, 'pbasis'));
            % Basis functions
            addParameter(p, 'BasisType', 'affine', ...
                @(x) any(strcmp(x, {'affine', 'poly', 'interp', 'custom'})));
            addParameter(p, 'BasisParametrization', {}, ...
                @(x) iscell(x));
            % Automatically sort and simplify
            addParameter(p, 'Simplify', true, ...
                @(x) islogical(x) && isscalar(x));
            % The user may either directly specify a timemap...
            addParameter(p, 'SchedulingTimeMap', [], ...
                @(x) isa(x, 'timemap'));
            % ...or build a timemap implicitly
            addParameter(p, 'SchedulingMap', 0);
            addParameter(p, 'SchedulingName', {});
            addParameter(p, 'SchedulingUnit', {});
            addParameter(p, 'SchedulingRange', {});
            addParameter(p, 'SchedulingRateBound', {});
            addParameter(p, 'SchedulingDimension', 1, ...
                @(x) isint(x) && x >= 0);
            addParameter(p, 'SchedulingDomain', 'ct', ...
                @(x) any(strcmpi(x, {'ct', 'dt', 'undefined'})));
            parse(p, varargin{:});
            SchedulingMap = p.Results.SchedulingMap;
            
            if ~any(strcmpi('SchedulingTimeMap', p.UsingDefaults))
                obj.timemap_ = p.Results.SchedulingTimeMap;
                assert( isempty(p.Results.SchedulingName) && ...
                        isempty(p.Results.SchedulingUnit) && ...
                        isempty(p.Results.SchedulingRange) && ...
                        isempty(p.Results.SchedulingRateBound) && ...
                        any(strcmpi('SchedulingDimension', p.UsingDefaults)) && ...
                        any(strcmpi('SchedulingDomain', p.UsingDefaults)),...
                            'LPVcore:pmat:Constructor:ToManyInputs',...
                        ['If SchedulingTimeMap is set, then none of the ', ...
                        'following may be set: ', ...
                        'Scheduling{Name/Unit/Range/RateBound/Dimension/Domain}']);
            else
                np = max([numel(p.Results.SchedulingName), ...
                    numel(p.Results.SchedulingUnit), ...
                    numel(p.Results.SchedulingRange), ...
                    numel(p.Results.SchedulingRateBound), ...
                    p.Results.SchedulingDimension]);
                obj.timemap_ = timemap(...
                    SchedulingMap, ...
                    p.Results.SchedulingDomain, ...
                    'Dimension', np, ...
                    'Name', p.Results.SchedulingName, ...
                    'Unit', p.Results.SchedulingUnit, ...
                    'RateBound', p.Results.SchedulingRateBound, ...
                    'Range', p.Results.SchedulingRange);
            end
            
            % Assign matrices
            if ismatrix(p.Results.mat)
                obj.matrices(:, :, 1) = p.Results.mat;
            else
                obj.matrices = p.Results.mat;
            end
            
            % Build basis functions if not explicitly passed
            if ~isempty(p.Results.bfuncs)
                % Basis functions were explicitly passed, check that
                % BasisType and BasisParametrization were not passed, too.
                assert( any(strcmpi('BasisType', p.UsingDefaults)) && ...
                        any(strcmpi('BasisParametrization', p.UsingDefaults)),...
                        'LPVcore:pmat:Constructor:InputMismatch',...
                        ['BasisType and BasisParametrization must not be ', ...
                            'used if basis functions are passed directly']);
                        
                assert(all(minNRho(p.Results.bfuncs{:}) <= obj.timemap.nrho), ...
                    'LPVcore:pmat:Constructor:InputMismatch',...
                    'rho must be smaller or equal to the size of the ext. scheduling signal.')
                obj.bfuncs = p.Results.bfuncs;
            else
                nrho = obj.timemap_.nrho;
                N = size(obj.matrices, 3);
                param = p.Results.BasisParametrization;
                if strcmp(p.Results.BasisType, 'affine')
                    % There can be at most nrho + 1 affine terms
                    if isempty(param)
                        assert( N <= nrho + 1, ...
                                'LPVcore:pmat:Constructor:InputMismatch',...
                                'Too many matrices specified for affine dependence.');
                        if N <= nrho
                            param = num2cell(linspace(1, N, N));
                        else
                            param = num2cell([linspace(1, nrho, nrho), 0]);
                        end
                    end
                    for i=1:N
                        if param{i} >= 1
                            assert( param{i} <= nrho,...
                                    'LPVcore:pmat:Constructor:InputMismatch',...
                                    ['The affine parametrization ', num2str(param{i}), ...
                                        ' exceeds the dimension of the extended ', ...
                                        'scheduling vector (', num2str(nrho), ').']);
                            obj.bfuncs{i} = pbaffine(param{i});
                        else
                            obj.bfuncs{i} = pbconst;
                        end
                    end
                elseif strcmp(p.Results.BasisType, 'poly')
                    % Each cell entry in the parametrization specifies degrees.
                    if isempty(param)
                        assert(nrho <= 1,...
                            	'LPVcore:pmat:Constructor:InputMismatch',...
                                ['Automatic generation of pmatrices ', ...
                                    'with polynomial dependence is currently only ', ...
                                    'possible for 1 scheduling variable. ', ...
                                    'Please specify the parametrization explicitly ', ...
                                    'using BasisParametrization.']);
                        for i=1:N
                            param{i} = i - 1;
                        end
                    end
                    for i=1:N
                        assert(numel(param{i}) <= nrho, ... %if numel(param{i}) > nrho
                            	'LPVcore:pmat:Constructor:InputMismatch',...
                                ['The number of elements of the parametrization ', ...
                                    num2str(param{i}), ...
                                    ' exceeds the dimension of the extended ', ...
                                    'scheduling vector (', num2str(nrho), ').']);
                        obj.bfuncs{i} = pbpoly(param{i});
                    end
                elseif strcmp(p.Results.BasisType, 'custom')
                    % Each cell entry in param. is a function handle.
                    assert(numel(param) == N, ...
                        'LPVcore:pmat:Constructor:InputMismatch',...
                        ['The number of custom basis functions in ', ...
                        'BasisParametrization must match the number of ', ...
                        'of matrix coefficients passed as the first argument']);
                    for i=1:N
                        if isempty(param{i})
                            obj.bfuncs{i} = pbconst;
                        else
                            obj.bfuncs{i} = pbcustom(param{i});
                        end
                    end
                elseif strcmp(p.Results.BasisType, 'interp')
                    % Each cell entry in param. is a function handle.
                    assert(numel(param) == N, ...
                        ['The number of cells containing gridded functions in ', ...
                        'BasisParametrization must match the number of ', ...
                        'of matrix coefficients passed as the first argument']);
                    for i=1:N
                        if isempty(param{i})
                            obj.bfuncs{i} = pbconst;
                        else
                            obj.bfuncs{i} = pbinterp(param{i}{:});
                        end
                    end
                end
            end
            
            if numel(obj.bfuncs) == 0
                obj.bfuncs = {pbconst};
                [n, m, ~] = size(obj.matrices);
                obj.matrices = zeros(n, m, 1);
            end
            if p.Results.Simplify
                obj = simplify_(obj);
            end
            lint_(obj);
        end
        
        function A = freeze(obj, p)
            %FREEZE Returns matrix for frozen value of the scheduling
            %parameter.
            %
            %   Syntax:
            %       A = freeze(obj, p)
            %
            %   Inputs:
            %       obj: n x m pmatrix object to evaluate
            %       p: 1-by-Np vector
            %
            %   Outputs:
            %       A: n-by-m constant matrix.
            %
            if nargin <= 1 && obj.Np == 0
                p = [];
            end
            rho = freeze(obj.timemap_, p);
            A = feval(obj, rho);
        end
        
        function [A, psi] = peval(obj, P, Ts)
            %PEVAL Returns matrix for a specified value of the scheduling
            %signal (not the extended scheduling signal).
            %
            %   Syntax:
            %       [B, psi] = peval(A, p)
            %       [B, psi] = peval(A, p, Ts)
            %
            %   Inputs:
            %       A: n-by-m pmatrix object to evaluate.
            %       p: N-by-np matrix.
            %       Ts: sampling time used for numerical calculation of the gradient. 
            %           Default: 1
            %
            %   Outputs:
            %       B: n-by-m-by-N numeric matrix.
            %       psi: N-by-nrho numeric matrix containing the
            %           trajectories of each basis function.
            %
            assert(isnumeric(P),'LPVcore:argNaN', 'P should be a numeric value')
            if nargin <= 2
                if isct(obj)
                    warning('Ts not specified for a CT pmatrix. Assuming 1');
                end
                Ts = 1;
            end
            rho = obj.timemap_.feval(P, Ts);
            if isempty(rho) && size(rho, 1) == 1
                A = freeze(obj);
                psi = [];
            else
                [A, psi] = feval(obj, rho);
            end
        end
        
        function [A, psi] = feval(obj, rho, basis)
            %FEVAL Returns matrix for a specified value of the extended scheduling
            %   signal.
            %
            %   Syntax:
            %       [A, psi] = feval(obj, rho)
            %       psi = feval(obj, rho, basis)
            %
            %   Inputs:
            %       obj: n-by-m pmatrix object to evaluate.
            %       rho: N-by-np matrix.
            %
            %   Outputs:
            %       B: n-by-m-by-N numeric array.
            %       psi: numeric matrix with N rows and as many columns as
            %           there are basis functions in obj.
            %
            % If basis = 'basis', then only evaluated basis functions are
            % returned. Otherwise, the matrix is returned.
            assert(isnumeric(rho),'LPVcore:argNaN', 'Rho should be a numeric value')
            N = size(rho, 1);
            % Calculate trajectory of basis functions
            psi = zeros(N, obj.numbfuncs);
            
            if size(rho, 2) == 1 
                rho = rho * ones(1, obj.timemap.nrho);
            else
                assert(size(rho, 2) == obj.timemap.nrho, ...
                    'LPVcore:pmat:argNumel', ...
                    '''rho'' must be an N-by-Nrho matrix');
            end

            for i=1:obj.numbfuncs
                psi(:, i) = feval(obj.bfuncs{i}, rho);
            end
            % Return if only basis is requested
            if nargin == 3 && strcmp(basis, 'basis')
                A = psi;
                return;
            end
            % Otherwise, calculate trajectory of matrix
            [n, m, o] = size(obj.matrices);
            C = reshape(obj.matrices, m,  o * n);
            PSI = kron(psi', eye(n));
            A = reshape(C * PSI, n, m, N);
        end
        
        function obj = pshift(obj, shiftAmount)
            %PSHIFT Shifts DT pmatrix by specified amount.
            %
            %   Syntax:
            %       B = pshift(A, shiftAmount)
            %
            %   Inputs:
            %       A: DT pmatrix object
            %       shiftAmount (int): amount of shifts (time-lag) to add
            %
            %   Outputs:
            %       B: time-shifted DT pmatrix object
            %
            assert(~isct(obj),...
                    'LPVcore:pmat:wrongTimemap',...
                    ['Attempting to use PSHIFT on CT pmatrix object.', ...
                        'Please use PDIFF instead.']);

            assert(isint(shiftAmount),...
                 'LPVcore:pmat:NonInteger',...
                 'Shift amount must be an integer');
            obj.timemap_ = shift(obj.timemap_, shiftAmount);
        end
        
        function obj = pdiff(obj, diffAmount)
            %PDIFF Differentiates CT pmatrix by specified amount.
            %
            %   Syntax:
            %       B = pdiff(A, diffAmount)
            %
            %   Inputs:
            %       A: DT pmatrix object
            %       diffAmount (non-negative int): amount of times to apply
            %           the differential operator.
            %
            %   Outputs:
            %       B: differentiated CT pmatrix object
            %
            %   Note: differentiation is only supported for non-custom
            %   basis functions!
            %
            assert(~isdt(obj),...
                    'LPVcore:pmat:wrongTimemap',...
                    ['Attempting to use PDIFF on DT pmatrix object.', ...
                        'Please use PSHIFT instead.']);
            assert(isint(diffAmount) && diffAmount >= 0, ...
                    'LPVcore:pmat:NonPositiveInteger',...
                	'Order or differentiation must be non-negative integer');
            assert(~iscustom(obj),...
                    'LPVcore:pmat:wrongBasisFunctions', ...
                    'Custom basis functions do not support differentiation.');
            for i=1:diffAmount
                % Add space for derivatives
                [obj.timemap_, idxA, ~] = union(obj.timemap_, shift(obj.timemap_, 1));
                % Re-index basis functions and calculate derivatives
                for j=numel(obj.bfuncs):-1:1
                    if isa(obj.bfuncs{j}, 'pbconst')
                        % Constants are set to 0
                        obj.matrices(:, :, j) = 0;
                    else
                        obj.bfuncs{j} = reidx(obj.bfuncs{j}, idxA);
                        obj.bfuncs{j} = diff(obj.bfuncs{j}, 1, obj.timemap_.Dimension);
                    end
                end
            end
        end
        
        function A = plus(A, B)
            % PLUS Add two pmatrix objects
            [A, B] = commonbfuncs_(A, B);
            A.matrices = A.matrices + B.matrices;
        end
        
        function A = uminus(A)
            % UMINUS Unitary minus
            A.matrices = -A.matrices;
        end
        
        function C = minus(A, B)
            % MINUS Subtract two pmatrix objects
            C = plus(A, -B);
        end
        
        function C = times(A, B)
            % TIMES Element-wise multiplication of two pmatrix objects
            [A, B] = pmatrix.commonrho_(A, B);
            [n, m, nA] = size(A.matrices);
            [n2, m2, ~] = size(B.matrices);
            assert(n==n2 || n==1 || n2==1,...%if n ~= n2 && ~(n == 1 || n2 == 1)
                'LPVcore:pmat:dimAgree', 'Matrix dimensions must agree.')
            if n < n2 && n ~= 0
                n = n2;
            end
            assert(m==m2 || m==1 || m2==1,... if m ~= m2 && ~(m == 1 || m2 == 1)
                'LPVcore:pmat:dimAgree', 'Matrix dimensions must agree.')
            if m < m2 && m~= 0
                m = m2;
            end

            nB = size(B.matrices, 3);
            mat = NaN(n, m, nA * nB);
            bf = cell(1, nA * nB);
            k = 1;
            % TODO: make more readable in case of size == 0
            if n == 0 || m == 0 || n2 == 0 || m2 == 0
                if n== 0
                    if m == 0
                        mat = zeros(n ,m ,nA*nB);
                    else
                        mat = zeros(n ,m2,nA*nB);
                    end
                else
                    if m == 0
                        mat = zeros(n2,m ,nA*nB);
                    else
                        mat = zeros(n2,m2,nA*nB);
                    end
                end
                for i=1:nA
                    for j=1:nB
                        bf{k} = A.bfuncs{i} * B.bfuncs{j};
                        k = k + 1;
                    end
                end
            else
                for i=1:nA
                    for j=1:nB
                        mat(:, :, k) = A.matrices(:, :, i) .* B.matrices(:, :, j);
                        bf{k} = A.bfuncs{i} * B.bfuncs{j};
                        k = k + 1;
                    end
                end
            end
            C = A;
            C.matrices = mat;
            C.bfuncs = bf;
            C = simplify_(C);
        end
        
        function C = mtimes(A, B)
            % MTIMES Matrix multiplication of two pmatrix objects
            [A, B] = pmatrix.commonrho_(A, B);
            [n, m] = size(A.matrices(:, :, 1) * B.matrices(:, :, 1));
            [~, ~, nA] = size(A.matrices);
            nB = size(B.matrices, 3);
            mat = NaN(n, m, nA * nB);
            bf = cell(1, nA * nB);
            k = 1;
            for i=1:nA
                for j=1:nB
                    mat(:, :, k) = A.matrices(:, :, i) * B.matrices(:, :, j);
                    bf{k} = A.bfuncs{i} * B.bfuncs{j};
                    k = k + 1;
                end
            end
            C = A;
            C.matrices = mat;
            C.bfuncs = bf;
            C = simplify_(C);
        end
        
        function C = mldivide(A, B)
            C = matop_(A, B, 'mldivide');
        end
        
        function C = mrdivide(A, B)
            C = matop_(A, B, 'mrdivide');
        end
        
        function C = rdivide(A, B)
            C = matop_(A, B, 'rdivide');
        end
        
        function C = ldivide(A, B)
            % LDIVIDE Left array divsion
            %   This function divides 
            %
            %   Syntax:
            %       C = ldivide(B, P)
            %       C = B .\ P
            %
            %   Inputs:
            %       B: non-parameter varying matrix
            %       P: pmatrix
            %
            %   Outputs:
            %       C: pmatrix after left array division of P with B.
            %
            C = matop_(A, B, 'ldivide');
        end
        
        function C = power(A, B)
            % POWER Element-wise power with integer power
            %
            %   Syntax:
            %       A^b
            %
            %   Inputs:
            %       A: pmatrix
            %       b (int): positive integer
            %
            %   Outputs:
            %       C: A^b
            assert(isscalar(B) && isnumeric(B) && isint(B) && B>= 0,...if ~isscalar(B) || ~isnumeric(B) || B ~= round(B) || B < 0
                    'LPVcore:pmat:argPosIntOnly',...
                    ['Matrix power can only be used with ', ...
                    'pmatrix for non-negative, scalar, integer B.']);
            if B == 0
                C = pmatrix(ones(size(A)), {pbconst}, 'SchedulingTimeMap', A.timemap);
                return;
            end
            C = A;
            for i=2:B
                C = times(C, A);
            end
        end
        
        function C = mpower(A, b)
            %MPOWER Matrix power with integer scalar
            %
            %   Syntax:
            %       A^b
            %
            %   Inputs:
            %       A: pmatrix
            %       b (int): positive integer
            %
            %   Outputs:
            %       C: A^b
            assert(isscalar(b) && isnumeric(b) && isint(b) && b>= 0,...if ~isscalar(b) || ~isnumeric(b) || b ~= round(b) || b < 0
                    'LPVcore:pmat:argPosIntOnly',...
                    ['Matrix power can only be used with ', ...
                        'pmatrix for non-negative, scalar, integer B.']);
            assert(issquare(A),...
                	'LPVcore:pmat:squareOnly',...
                    ['Matrix power can only be used on square matrices.',...
                    'To perform elementwise multiplication use ''.*''.'])
            if b == 0
                C = pmatrix(eye(size(A, 1)), {pbconst}, 'SchedulingTimeMap', A.timemap);
                return;
            end
            C = A;
            for i=2:b
                C = mtimes(C, A);
            end
        end
        
        function C = kron(A, B)
            %KRON Kronecker tensor product for pmatrix
            %
            %   Syntax:
            %       C = kron(A, B)
            %
            %   Inputs:
            %       A, B: pmatrix objects
            %
            %   Outputs:
            %       C: pmatrix object
            [A, B] = pmatrix.commonrho_(A, B);
            [nA, mA, oA] = size(A);
            [nB, mB, oB] = size(B);
            mat_ = zeros(nA * nB, mA * mB, oA * oB);
            bf_ = cell(1, oA * oB);
            tm_ = A.timemap;
            k = 1;
            for i=1:oA
                for j=1:oB
                    mat_(:, :, k) = kron(A.matrices(:, :, i), ...
                        B.matrices(:, :, j));
                    bf_{k} = A.bfuncs{i} * B.bfuncs{j};
                    k = k + 1;
                end
            end
            C = pmatrix(mat_, bf_, 'SchedulingTimeMap', tm_);
        end
        
        function varargout = size(obj, varargin)
            % SIZE Return size of pmatrix
            %
            %   Syntax:
            %       sz = size(P)
            %       szdim = size(P, dim)
            %       [sz1, sz2, sz3] = size(__)
            %
            %   Inputs:
            %       P: pmatrix objects
            %       dim: (int) dimension along which to return the size.
            %       Valid values are 1 and 2 for the sizes of the
            %       matrix-valued coefficients, and 3 for the number of
            %       basis functions.
            %
            %   Outputs:
            %       sz: row vector with size of P along first two
            %       dimensions.
            %       szdim: size along the requested dimension.
            %       sz1, ..., sz3: size along each of the three dimensions
            %       returned as separate integers.
            %
            sz = [size(obj.matrices, 1), size(obj.matrices, 2), ...
                size(obj.matrices, 3)];
            if nargin == 2
                dim = varargin{1};
                if dim > 3
                    varargout{1} = 1;
                else
                    varargout{1} = sz(dim);
                end
            elseif nargout <= 1
                varargout{1} = sz(1:2);
            elseif nargout == 2
                varargout{1} = sz(1);
                varargout{2} = sz(2);
            elseif nargout == 3
                varargout{1} = sz(1);
                varargout{2} = sz(2);
                varargout{3} = sz(3);
            end
        end
        
        function v = isempty(obj)
            % ISEMPTY Returns whether any pmatrix dimension is 0
            v = size(obj, 1) == 0 || size(obj, 2) == 0;
        end
        
        function B = ctranspose(A)
            %CTRANSPOSE Overload complex conjugate transpose.
            %
            %   Example:
            %       F = P'
            [n, m, o] = size(A);
            mat_ = zeros(m, n, o);
            for i=1:size(A, 3)
                mat_(:, :, i) = A.matrices(:, :, i)';
            end
            B = pmatrix(mat_, A.bfuncs, 'SchedulingTimeMap', A.timemap);
        end
        
        function B = transpose(A)
            %TRANSPOSE Transpose operation
            %
            %   Example:
            %       F = P.'
            [n, m, o] = size(A);
            mat_ = zeros(m, n, o);
            for i=1:size(A, 3)
                mat_(:, :, i) = A.matrices(:, :, i).';
            end
            B = pmatrix(mat_, A.bfuncs, 'SchedulingTimeMap', A.timemap);
        end
        
        function C = vertcat(varargin)
            % VERTCAT Vertical concatenation
            C = cat(1, varargin{:});
        end
        
        function C = horzcat(varargin)
            % HORZCAT Horizontal concatenation
            C = cat(2, varargin{:});
        end
        
        function [C, IA, IB] = cat(dim, varargin)
            % CAT Concatenation
            assert(nargout <= 1 || nargin <= 3,...
                	'LPVcore:pmat:InOutMismatch',...
                    ['IA and IB are only available when concatenating ', ...
                        '2 pmatrix objects.']);
            assert(isint(dim), '''dim'' must be an integer');
            assert(dim >= 1 && dim <= 3, ...
                'Concatenation along dim=%i not supported', dim);
            [varargin{1:nargin-1}] = pmatrix.commonrho_(varargin{:});
            C = varargin{1};
            
            % iteratively concatenate pairs of pmatrix objects
            for i=2:nargin-1
                [C, IA, IB] = cat_(dim, C, varargin{i});
            end
        end
        
        function C = blkdiag(varargin)
            % BLKDIAG Block diagonal concatenation
            [varargin{:}] = pmatrix.commonrho_(varargin{:});
            C = varargin{1};
            
            % iteratively blkdiag pairs of pmatrix objects
            for i=2:nargin
                C = blkdiag_(C, varargin{i});
            end
        end
        
        function v = diag(v, k)
            %DIAG Create diagonal pmatrix or get diagonal elements of
            %pmatrix
            %
            %   Syntax:
            %       D = diag(v)
            %
            %       x = diag(P)
            %
            %   Inputs:
            %       v: vector of pmatrix scalars
            %       P: pmatrix objects
            %
            %   Outputs:
            %       D: diagonal pmatrix object
            %       x: column vector of pmatrix scalars
            %
            if nargin <= 1
                k = 0;
            end
            for i=1:size(v, 3)
                mat(:, :, i) = diag(v.matrices(:, :, i), k); %#ok<AGROW>
            end
            v.matrices = mat;
        end
        
        function c = eq(A, B)
            %EQ: necessary condition for equality of 2 pmatrix objects
            
            % Equality is checked at rho = pi (hence, just a necessary
            % condition)
            c = max(max(abs(feval(A - B, pi)))) <= 1E-10;
        end
        
        function c = ne(A, B)
            % NE Sufficient condition for inequality of 2 pmatrix objects
            c = ~eq(A, B);
        end
        
        function c = issquare(A)
            % ISSQUARE Returns whether the pmatrix is square
            [n, m] = size(A);
            c = (n == m);
        end
        
        function c = isconst(A, tol)
            % ISCONST Returns whether the pmatrix object is constant
            if nargin == 1; tol = eps; end
            c = true;
            if isempty(A); return; end
            for i=1:A.numbfuncs
                if ~isa(A.bfuncs{i}, 'pbconst') && ...
                    max(abs(A.matrices(:, :, i)),[],'all') > tol
                    c = false;
                    return;
                end
            end
        end
        
        function c = isaffine(A, tol)
            % ISAFFINE Returns whether the pmatrix object has affine dependency on
            % the scheduling variable
            if nargin == 1; tol = eps; end
            c = true;
            if isempty(A); return;  end
                
            for i=1:A.numbfuncs
                if ~isa(A.bfuncs{i}, 'pbaffine') && ...
                    max(abs(A.matrices(:, :, i)),[],'all') > tol
                    c = false;
                    return;
                end
            end
        end

        function c = isinterp(A, tol)
            % ISINTERP Returns whether the pmatrix object has interp dependency on
            % the scheduling variable
            if nargin == 1; tol = eps; end
            c = true;
            if isempty(A); return;  end
                
            for i=1:A.numbfuncs
                if ~isa(A.bfuncs{i}, 'pbinterp') && ...
                    max(abs(A.matrices(:, :, i)),[],'all') > tol
                    c = false;
                    return;
                end
            end
        end
        
        function c = ispoly(obj, tol)
            % ISPOLY Returns whether a pmatrix object has a polynomial
            %dependency on scheduling variable
            if nargin == 1; tol = eps; end
            c = true;
            if isempty(obj); return; end
            for i = 1:obj.numbfuncs
                if ~isa(obj.bfuncs{i},'pbpoly') && ...
                    max(abs(obj.matrices(:, :, i)), [],'all') > tol
                    c = false;
                    return
                end
            end
        end
        
        function c = iscustom(obj, tol)
            %Returns whether a function of the pmatrixobject has custom
            %dependency on a scheduling variable
            if nargin == 1; tol = eps; end
            c = false;
            if isempty(obj); return; end
            for i = 1:obj.numbfuncs
                if isa(obj.bfuncs{i},'pbcustom') && ...
                    max(abs(obj.matrices(:, :, i)), [],'all') > tol
                    c = true;
                    return
                end
            end
        end
        
        function c = isdt(obj)
            %Returns whether a the p-matrix is discrete time dependent.
            c = strcmp(obj.timemap.Domain, 'dt');
        end
        
        function c = isct(obj)
            %Returns whether a the p-matrix is continous time dependent.
            c = strcmp(obj.timemap.Domain, 'ct');
        end

        function v = isdiag(obj)
            % Returns whether all coefficients of the pmatrix are diagonal
            v = true;
            for i=1:size(obj, 3)
                if ~isdiag(obj.matrices(:, :, i))
                    v = false;
                    break;
                end
            end
        end
        
        function val = get.timemap(obj)
            val = obj.timemap_;
        end
        
        function obj = set.timemap(obj, val)
            % Perform re-indexing of the basis functions based on the new
            % timemap
            [obj.timemap_, idx, ~] = union(obj.timemap_, val);
            o = size(obj.matrices, 3);
            for j=1:o
                obj.bfuncs{j} = ...
                    obj.bfuncs{j}.reidx(idx);
            end
        end
        
        function idx = getIdxFromNewTimemap(obj, val)
            % GETIDXFROMNEWTIMEMAP Returns the re-indexing that would occur
            % if a new timemap were set (but does not update the timemap).
            [~, idx] = union(obj.timemap_, val);
        end
        
        function disp(obj)
            % DISP Print object information to screen
            if isempty(obj)
                disp('Empty scheduling dependent matrix');
                return
            end
            sz1 = size(obj,1);
            sz2 = size(obj,2);
            domain = [];
            if isct(obj)
                domain = 'Continuous-time ';
            elseif isdt(obj)
                domain = 'Discrete-time ';
            end

            symbolRepr = symbstr(obj);
            if sz1 == 1 && sz2 == 1
                disp( [ domain , 'scheduling dependent matrix'] )
                disp( ' ' )
                fprintf(['\t', symbolRepr])
                fprintf('\n\n');
            else
                disp( [ domain , 'scheduling dependent ' ,num2str(sz1), 'x', num2str(sz2),' matrix'] )
                disp( ' ' )
                fprintf(['\t', symbolRepr])
                fprintf('\n\n')
                dispcoeff(obj, 'A');
            end
        end
        
        function [A, I] = simplify(A, tol)
        %SIMPLIFY Sort and remove duplicate basis functions.
            %
            %   Syntax:
            %       [B, I] = simplify(A, tol)
            %
            %   Inputs:
            %       A: pmatrix to simplify.
            %       tol (double): matrix coefficients for which the magnitude of the element with 
            %           the largest magnitude is smaller than tol are removed. Default: -1 (no removal)
            %
            %   Outputs:
            %       B: simplified pmatrix.
            %       I: index vector indicating the new position of any
            %           remaining basis functions in B.
            %
            if nargin <= 1
                tol = -1;
            end
            [A, I] = simplify_(A, tol);
        end
    
        function S = sum(obj, varargin)
            % SUM Sum operation on matrix coefficients
            %
            % Syntax: see built-in SUM function.
            %
            % Note: this function may exhibit non-MATLAB-like behavior
            % if the pmatrix object is empty.
            %
            for i=1:size(obj,3)
                %TODO: memory allocation can be better
                S_mat(:,:,i) = sum(obj.matrices(:,:,i),varargin{:}); %#ok<AGROW>
            end
            S = obj;
            S.matrices = S_mat;
        end
        
        function obj = reshape(obj, varargin)
            % RESHAPE Reshape matrix coefficients
            %
            % Syntax: see built-in RESHAPE function.
            %
            assert(nargin<=3 && size(varargin{1},1) <= 1 && size(varargin{1},2) <= 2 ,...if nargin > 3 || size(varargin{1},2)>2 ||  size(varargin{1},1) > 1
                	'LPVcore:DimConvention', ...
                    "reshape only accepts two-dimensional size inputs");
            if nargin == 2
                obj.matrices = reshape(obj.matrices, [varargin{:}, obj.numbfuncs]);
                return
            end
            obj.matrices = reshape(obj.matrices, varargin{:}, obj.numbfuncs);
        end
        
        function obj = slice(obj, i1, i2)
            % SLICE Slice pmatrix between two indices
            %
            % Slices pmatrix phi1 * P1 + ... + phin * Pn between indices i1
            % and i2 so that the resulting pmatrix is phi{i1} * P{i1} + ...
            % + phi{i2} * P{i2}.
            %
            % Syntax:
            %   B = slice(A, i1)
            %   B = slice(A, i1, i2)
            %
            % Inputs:
            %   A: pmatrix to slice
            %   i1: (int) start index (inclusive)
            %   i2: (int) end index (inclusive), or use 'end' to denote the
            %       last index. If omitted, i2 is set to i1 (slice contains
            %       only one matrix coefficient).
            %
            % Outputs:
            %   B: sliced pmatrix
            %

            %% Input validation
            narginchk(2, 3);
            nargoutchk(1, 1);
            % n: max index
            n = size(obj, 3);
            assert(isint(i1) && i1 >= 1 && i1 <= n, ...
                '''i1'' (%i) must be between 1 and %i', i1, n);
            if nargin <= 2
                i2 = i1;
            end
            assert(isint(i2) || strcmp(i2, 'end'), '''i2'' must be integer or ''end''');
            if strcmp(i2, 'end')
                i2 = n;
            end
            assert(isint(i2) && i2 >= i1 && i2 <= n, ...
                '''i2'' (%i) must be between ''i1'' (%i) and %i', i2, i1, n);

            %% Slice
            obj.matrices = obj.matrices(:, :, i1:i2);
            obj.bfuncs = obj.bfuncs(i1:i2);
            obj.numbfuncs = i2 - i1 + 1;
        end

        function n = numel(obj)
            [sz1, sz2] = size(obj.matrices(:,:,1));
            n = sz1 * sz2;
        end
    end

    methods(Static)
        function obj = empty(varargin)
            % Creates an empty pmatrix with empty matrices with default
            % pmatrix values for all other variables.
            obj = pmatrix(double.empty(varargin{:}));
        end
    end    
    
    % Get methods
    methods
        function val = get.Np(obj)
            val = obj.timemap.np;
        end
    end
    
    methods (Hidden)
        function s = symbstr(obj, prefix)
            % Return symbolic string of pmatrix object
            if nargin <= 1
                prefix = 'A';
            end
            s = '';
            Nbf = numel(obj.bfuncs);
            for i=1:Nbf
                if ~isa(obj.bfuncs{i}, 'pbconst')
                    bf = ['*(', str(obj.bfuncs{i}, obj.timemap.RhoName), ')'];
                else
                    bf = '';
                end
                if isscalar(obj)
                    tmpStr = [ '<strong>' , num2str(obj.matrices(:,:,i)) ,'</strong>' , bf];
                else
                    tmpStr = [ '<strong>', prefix, num2str(i) ,'</strong>' , bf];
                end
                if i ~= Nbf
                    tmpStr = [tmpStr , ' + ']; %#ok<AGROW>
                end
                s = [s, tmpStr]; %#ok<AGROW>
            end
        end
        
        function dispcoeff(obj, prefix)
            % Display coefficients
            if nargin <= 1
                prefix = 'A';
            end
            for i=1:numel(obj.bfuncs)
                disp([prefix, num2str(i), ' ='])
                disp(obj.matrices(:,:,i))
            end
        end
        
        function A = c2d(A)
            % C2D Convert continuous-time pmatrix object to discrete-time
            %
            %   Used by lpvlfr/c2d to convert Delta from CT to DT.
            %
            A.timemap_.Domain = 'dt';
        end

        function [A, B] = commonbfuncs_(A, B)
            %assumes simplify_ed before call, thus equal bfuncs in same
            %order
            [sz11, sz12] = size(A);
            [sz21, sz22] = size(B);
            [A, B] = pmatrix.commonrho_(A,B);
            Ind1 = 1:A.numbfuncs;
            Ind2 = 1:B.numbfuncs;
            Indi = [];
            Indj = [];
            tmp1 = 1;
            for i = 1:A.numbfuncs
               for j = tmp1:B.numbfuncs
                   if A.bfuncs{i} == B.bfuncs{j}
                       Ind1(Ind1 == i) = [];  
                       Indi = [Indi , i]; %#ok<AGROW>
                       Ind2(Ind2 == j) = [];
                       Indj = [Indj , j]; %#ok<AGROW>
                       tmp1 = j;
                   end
               end
            end
            A.matrices = cat(3,A.matrices(:,:,Ind1),A.matrices(:,:,Indi), zeros(sz11,sz12,numel(Ind2)));
            A.bfuncs = [A.bfuncs([Ind1, Indi]), B.bfuncs(Ind2)];
            A.numbfuncs = A.numbfuncs + numel(Ind2);
            B.matrices = cat(3, zeros(sz21,sz22,numel(Ind1)), B.matrices(:,:,Indj), B.matrices(:,:,Ind2));
            B.bfuncs = A.bfuncs;
            B.numbfuncs = A.numbfuncs;
        end
    end
    
    methods (Access='private')
        function C = matop_(A, B, opType)
            %MATOP_: Apply specified matrix operation to pmatrix object.
            %
            %   Syntax:
            %       C = matop_(A, B, opType)
            %
            %   Inputs:
            %       A: pmatrix object
            %       B: numeric scalar or appropriately sized matrix
            %       opType (char vector): operation type. Choose from:
            %           'rdivide', 'ldivide', 
            %           'mrdivide', 'mldivide'
            %
            if strcmp(opType, 'rdivide') || strcmp(opType, 'mrdivide')
                % A is pmatrix
                C = A;
                assert(isnumeric(B),'LPVcore:argNaN','Operation can only be performed with numeric matrices.');
                for i=1:size(C, 3)
                    AMat = A.matrices(:, :, i);
                    switch(opType)
                        case 'rdivide'
                            CMats(:, :, i) = AMat ./ B; %#ok<AGROW>
                        case 'mrdivide'
                            CMats(:, :, i) = AMat / B; %#ok<AGROW>
                    end
                end
                C.matrices = CMats;
            else
                % B is pmatrix
                C = B;
                assert(isnumeric(A),'LPVcore:argNaN','Operation can only be performed with numeric matrices.');
                for i=1:size(C, 3)
                    BMat = B.matrices(:, :, i);
                    switch(opType)
                        case 'ldivide'
                            CMats(:, :, i) = A .\ BMat; %#ok<AGROW>
                        case 'mldivide'
                            CMats(:, :, i) = A \ BMat; %#ok<AGROW>
                    end
                end
                C.matrices = CMats;
            end
        end

        function lint_(A)
            %LINT_ Validates object properties
            mat_ = A.matrices;
            bfuncs_ = A.bfuncs;
            
            assert(size(mat_, 3) == numel(bfuncs_));
        end
        
        function [A, I_i2d] = simplify_(A, tol)
            %SIMPLIFY_: sort and remove duplicate basis functions.
            %
            %   Syntax:
            %       A = simplify_(A, tol)
            %
            %   Inputs:
            %       A: pmatrix to simplify.
            %       tol (double): matrices with max. element smaller than
            %           tol are removed. Default: -1 (no removal)
            
            %TODO: 
            %constant basisfunctions poly(with multiple lines) and constant customs
            % are not collapsed into a constant 1 but kept as a seperate bf
            
            %TODO:
            %in case of same basis functions (same sortKey) but different dependenceType
            %resolve to 'best' instead of first (e.g., prioritize pbpoly over pbcustom
            % so automatic differentiation is available).
            
            if nargin == 1
                tol = -1;
            end
            n = size(A, 3);
            [A, I] = sort_(A);      % note: gives forward sort index
            I_i2s(I) = (1:n)';  	% reverse sort index: input to sorted
            I_s2d = (1:n)';         % sorted to duplicates removed
            
            % First check is to see which entries are duplicate
            % Running time: O(n^2)
            toRemove = false(n, 1);
            for i=n:-1:1
                bf = A.bfuncs{i};
                for j=i+1:n
                    cbf = A.bfuncs{j};
                    if bf == cbf
                        toRemove(j) = true;
                        I_s2d(j:end) = I_s2d(j:end) - 1;
                    end
                end
            end
            I_i2d = I_s2d(I_i2s);
            
            % Add up all duplicate values
            % Running time: O(n^2)
            for i=1:n
                if ~toRemove(i)
                    bf = A.bfuncs{i};
                    for j=i+1:n
                        cbf = A.bfuncs{j};
                        if bf == cbf
                            A.matrices(:, :, i) = A.matrices(:, :, i) + ...
                                A.matrices(:, :, j);
                        end
                    end
                end
            end
            
            % Remove small entries
            % Running time: O(n)
            if tol > 0
                for i=1:n
                    mat = A.matrices(:, :, i);
                    bf = A.bfuncs{i};
                    if ~isa(bf, 'pbconst') && ...
                            (~isempty(mat) && max(max(abs(mat))) < tol)
                        toRemove(i) = true;
                        I(i) = NaN;
                    end
                end
            end
            A.matrices(:, :, toRemove) = [];
            A.bfuncs(toRemove) = [];
            [~, ~, A.numbfuncs] = size(A.matrices);
        end
        
        function [A, I] = sort_(A)
            %SORT_ Sorts (basis function, matrix) pairs.
            N = size(A, 3);
            keys(N) = "";
            % Each pbasis subclass implements the SORTKEY method that
            % returns a key (integer). This set of keys is then sorted to
            % sort the (basis function, matrix) pairs.
            for i=1:N
                keys(i) = A.bfuncs{i}.sortKey;
            end
            [~, I] = sort(keys);
            A.matrices = A.matrices(:, :, I);
            A.bfuncs = A.bfuncs(I);
        end
        
        function [C, IA, IB] = cat_(dim, A, B)
            %CAT_ Concatenates 2 pmatrix objects with common ext.
            %scheduling parameter.
            %
            %   Synatx:
            %       [C, IA, IB] = cat_(dim, A, B)
            %
            %   Inputs:
            %       dim (int): dimension along which to concatenate
            %       A, B: pmatrix objects to concatenate. Assumptions:
            %           * same ext. scheduling variable (commonrho)
            %
            %   Outputs:
            %       C: concatenated pmatrix object
            %       IA, IB: index vector describing rearrangement of basis
            %           functions for A and B.
            %
            IA = []; IB = [];
            if isempty(A)
                C = B;
                return;
            end
            if isempty(B)
                C = A;
                return;
            end
            % Intermediate result is always stored in pmatrix with N basis
            % functions, with N being the sum of basis functions of A and
            % B. After simplification, the duplicate basis functions are
            % removed.
            [nA, mA, oA] = size(A);
            [nB, mB, oB] = size(B);
            % If dim ~= 3, 3rd dimension must be made equal. Since we do
            % not assume A and B to have common basis functions, zeros are
            % added to the end of the 3rd dimension of A and to the start
            % of the 3rd dimension of B.
            % TODO: improve speed of cat_ by assuming common bfuncs (need
            % to be adjusted in cat)
            if dim ~= 3
                matA_ = cat(3, A.matrices, zeros(nA, mA, oB));
                matB_ = cat(3, zeros(nB, mB, oA), B.matrices);
            else
                matA_ = A.matrices;
                matB_ = B.matrices;
            end
            mat_ = cat(dim, matA_, matB_);
            bf_ = [A.bfuncs, B.bfuncs];
            tm_ = A.timemap;
            
            C = pmatrix(mat_, bf_, 'SchedulingTimeMap', tm_, ...
                'Simplify', false);
            [C, I] = simplify(C);
            IA = I(1:size(A, 3));
            IB = I((size(A, 3)+1):end);
        end
        
        function C = blkdiag_(A, B)
            if isempty(B)
                C = A;
                return;
            end
            if isempty(A)
                C = B;
                return;
            end
            [nA, mA, ~] = size(A);
            [nB, mB, ~] = size(B);
            tm = A.timemap;
            C1 = cat_(2, A, pmatrix(zeros(nA, mB), {pbconst}, 'SchedulingTimeMap', tm));
            C2 = cat_(2, pmatrix(zeros(nB, mA), {pbconst}, 'SchedulingTimeMap', tm), B);
            C = cat_(1, C1, C2);
        end
        
        function B = subsref_(A, i, j)
            %SUBSREF_ Wrapper around SUBSREF for use in class methods.
            %
            %   For more info: https://nl.mathworks.com/matlabcentral/answers/101830-why-is-the-subsref-method-not-called-when-used-from-within-a-method-of-a-class
            %
            S.type = '()';
            S.subs{1} = i;
            S.subs{2} = j;
            B = subsref(A, S);
        end
        
        function C = subsasgn_(A, i, j, B)
            %SUBSASGN_ Wrapper around SUBSASGN for use in class methods
            S.type = '()';
            S.subs{1} = i;
            S.subs{2} = j;
            C = subsasgn(A, S, B);
        end
        
        function A = removebfunc_(A, i)
            % Removes the i'th basis function
            A.bfuncs(i) = [];
            A.matrices(:, :, i) = [];
            A.numbfuncs = A.numbfuncs - 1;
        end
    end
end

