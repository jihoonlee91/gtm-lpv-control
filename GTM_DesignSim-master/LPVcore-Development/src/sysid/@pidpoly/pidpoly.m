classdef pidpoly
    %PIDPOLY Helper class for lpvidpoly
    %
    %   An LPVIDPOLY system consists of 5 polynomials: A, B, C, D and F.
    %   These polynomials have scheduling-dependent coefficients: PMATRIX
    %   objects. Each coefficient represents a time-delay or differentiation,
    %   i.e.:
    %
    %       A(p, q) := A0(p) + A1(p) q^{-1} + ... + AN(p) q^{-N}
    %
    %   The above structure is captured by PIDPOLY. It provides
    %   convenient methods for concatenation, retrieving information on the
    %   parameter set, and some operations such as subtraction by 1 (A -
    %   1).
    %
    %   You generally do not need to manipulate PIDPOLY objects directly:
    %   the logic is handled by the higher-level LPVIDPOLY object.
    %
    %   Syntax:
    %       A = pidpoly(Mat, Free, Min, Max, monic, Name, Value)
    %
    %   Inputs:
    %       Mat: cell array of PMATRIX objects A0, ..., AN
    %       Free: cell array of logical arrays indicating which parameters in the
    %           pmatrix coefficients A0, ..., AN are free. [] creates the default.
    %           The size of each array must match the size of the
    %           corresponding coefficient.
    %       Min, Max: cell array of double arrays indicating the range of the
    %           free parameters. [] creates the default (-Inf to +Inf).
    %       monic: logical indicating whether the object is monic, i.e.,
    %           has a leading identity coefficient. This coefficient does
    %           not contribute to the number of identifiable parameters
    %           (see NPARAMS).
    %
    %   Name, Value pairs:
    %       'ZeroIsNonFree': logical indicating whether a 0 entry in Mat
    %       constitutes a non-free parameter  (e.g. {A0, 0, A2}). Default:
    %       true.
    %
    
    properties
        % Boolean indicating whether zero entries in A are non-free parameters. Default: true
        ZeroIsNonFree = true
    end
    
    properties (Dependent)
        % Cell array of pmatrix coefficients. The polynomial is represented
        % as Mat{1} + Mat{2} * q + Mat{3} * q^2 + ... with "q" either
        % the time shift (DT) or differentiation (CT) operator.
        Mat
        % Cell array of logical arrays indicating which of the coefficients
        % are free model parameters.
        Free
        % Cell array of double arrays indicating the minimum value of each
        % coefficient.
        Min
        % Cell array of double arrays indicating the maximum value of each
        % coefficient.
        Max
        % Number of coefficients, i.e., size of Mat property.
        N
        SchedulingTimeMap
        Value           % Current values of the matrices in Mat
        MatNoMonic      % Alias to Mat without leading term
        FreeNoMonic     % Alias to Free without leading term
        MinNoMonic      % Alias to Min without leading term
        MaxNoMonic      % Alias to Max without leading term
        ValueNoMonic    % Alias to Value without leading term
    end
    
    properties (Hidden)
        Mat_
        Free_
        Min_
        Max_
        MustBeMonic_
    end
    
    methods
        function obj = pidpoly(Mat, Free, Min, Max, MustBeMonic, ZeroIsNonFree)
            %PIDPOLY Construct an instance of this class
            %
            %   Syntax:
            %
            %       F = pidpoly(Mat, Free, Min, Max)
            %       F = pidpoly(Mat, [], [], [], monic, ZeroIsNonFree)
            %
            %   Inputs:
            %       
            %       Mat: cell array of pmatrix objects.
            %       Free: empty or cell array of logicals.
            %       Min / Max: empty or cell array of doubles.
            %       MustBeMonic: logical indicating whether filter is monic and must remain monic.
            %           Default: false
            %      ZeroIsNonFree: logical indicating whether zero pmatrix
            %           objects are set to non-free parameters. Default: true
            %
            if nargin <= 4; MustBeMonic = false; end
            if nargin <= 5; ZeroIsNonFree = true; end
            obj.MustBeMonic_ = MustBeMonic;
            obj.Mat = Mat;
            obj.ZeroIsNonFree = ZeroIsNonFree;
            obj = defaultStruct_(obj, MustBeMonic);
            if nargin <= 1; Free = []; end
            if nargin <= 2; Min = []; end
            if nargin <= 3; Max = []; end
            if ~isempty(Free); obj.Free = Free; end
            if ~isempty(Min); obj.Min = Min; end
            if ~isempty(Max); obj.Max = Max; end
        end
        
        function D = parder(obj)
            % PARDER Return derivatives with respect to free parameters
            %
            %   Syntax:
            %       D = parder(A)
            %
            %   Inputs:
            %       A: pidpoly object
            %
            %   Outputs:
            %       D: cell array of pidpoly objects representing the partial derivative
            %       of A for each free parameter.
            %
            
            % Loop through filters
            D = cell(1, nparams(obj, 'free'));
            Z = {};
            DCount = 1;
            for i=1:obj.N
                free = obj.Free{i};  % 3-dimensional double
                mat = obj.Mat{i};    % pmatrix
                [n, m, o] = size(free);
                for j=1:o
                    bfunc = mat.bfuncs{j};
                    for k=1:n
                        for l=1:m
                            if free(k, l, j)
                                matMasked = zeros(n, m);
                                matMasked(k, l) = 1;
                                P = pmatrix(matMasked, {bfunc}, ...
                                    'SchedulingTimeMap', mat.timemap);
                                D{DCount} = pidpoly([Z(:)', {P}]);
                                DCount = DCount + 1;
                            end
                        end
                    end
                end
                Z = [Z{:}, {zeros(n, m)}];
            end
        end

        % TODO: merge functionality with APPLYDELTAP
        function [obj, p] = setpvec(obj, p)
            % SETPVEC Set free parameters
            %
            %   Syntax:
            %       [obj, p] = setpvec(obj, p)
            %
            %   Inputs:
            %       p: N-by-1 vector of free parameter values.
            %           If N > nparams(obj, 'free'), then only the first
            %           nparams(obj, 'free') elements will be used.
            %
            %   Outputs:
            %       delta: (N - nparams(obj, 'free'))-by-1 vector of free
            %           parameters that were not used to update this particular
            %           filter.
            %
            
            % Input validation
            if numel(p) < nparams(obj, 'free')
                error('Too few parameters were provided to update values.');
            end
            % Loop through filters
            for i=1:obj.N
                free = obj.Free{i};
                mat = obj.Mat{i};
                [~, ~, o] = size(free);
                % Update pmatrix values
                % Loop through basis functions
                for j=1:o
                    % Loop through columns
                    for k=1:size(obj, 1)
                        for l=1:size(obj, 2)
                            if free(k, l, j)
                                % Assign parameter and remove
                                mat.matrices(k, l, j) = p(1);
                                p(1) = [];
                            end
                        end
                    end
                end
                % Write pmatrix with updated values back
                obj.Mat{i} = mat;
            end
        end

        function [obj, delta] = applydeltap(obj, delta)
            % APPLYDELTAP Update free parameters
            %
            %   Syntax:
            %       [obj, delta] = applydeltap(obj, delta)
            %
            %   Inputs:
            %       delta: N-by-1 vector of free parameter
            %       updates (corresponding e.g. to a single gradient step).
            %       If N > nparams(obj, 'free'), then only the first
            %       nparams(obj, 'free') elements will be used.
            %
            %   Outputs:
            %       delta: (N-nparams(obj, 'free'))-by-1 vector of free
            %       parameters that were not used to update this particular
            %       filter.
            
            % Input validation
            if numel(delta) < nparams(obj, 'free')
                error('Too few parameters were provided to update values.');
            end
            assert(isscalar(obj), '''setpvec'' can only be used on scalar polynomials');
            % Loop through filters
            for i=1:obj.N
                free = obj.Free{i};
                mat = obj.Mat{i};
                [~, ~, o] = size(free);
                % Update pmatrix values
                for j=1:o
                    if free(1, 1, j) %TODO: update for MIMO
                        mat.matrices(1, 1, j) = ...
                            mat.matrices(1, 1, j) + delta(1);
                        delta(1) = [];
                    end
                end
                % Write pmatrix with updated values back
                obj.Mat{i} = mat;
            end
        end
        
        function B = truncate(A, n)
            % TRUNCATE Truncates filter to n terms
            %
            %   Example:
            %
            %       A := A0 + A1 q^{-1} + ... + A(n+m) q^{-n-m}
            %
            %   Calling truncate(A, n) leads to:
            %
            %       A0 + A1 q^{-1} + ... + An q^{-n}
            %
            %   Syntax:
            %       B = truncate(A, n)
            %
            %   Inputs:
            %       A: pidpoly
            %       n: number of terms to keep
            %
            mat = A.Mat_(1:n);
            free = A.Free_(1:n);
            min = A.Min(1:n);
            max = A.Max(1:n);
            B = pidpoly(mat, free, min, max);
        end
        
        function varargout = subsref(obj, S)
            %SUBSREF Subscripted referencing for pidpoly objects.
            %
            %   F = P(i:j, k:l)
            %
            %   Inputs:
            %       P: indexed pidpoly object.
            %       S: indexing structure.
            %
            %   Outputs:
            %       F: result of indexing expression.
            %
            %   This method overloads the builtin SUBSREF functionality for the case of
            %   2-dimensional parentheses indexing. In this case, it indexes the
            %   output and input channels of the polynomial.
            %
            
            % Only subsref of the following form are overloaded:
            %
            %   indexing: P(x, y)
            %
            
            if numel(S) == 1 && strcmp(S.type, '()') && numel(S.subs) <= 2 ...
                    && ~(numel(S.subs) == 1 && isa(S.subs{1}, 'char'))  % ':'
                rows = S.subs{1};
                columns = S.subs{2};
                varargout{1} = slice(obj, rows, columns);
            else
                [varargout{1:nargout}] = builtin('subsref', obj, S);
            end
        end

        function B = slice(A, n, m)
            % SLICE Slices filter by taking n output channels and m input
            % channels.
            %
            %   Syntax:
            %       B = slice(A, n, m)
            %       B = slice(A, [], m)
            %       B = slice(A, n, [])
            %
            %   Inputs:
            %       A: pidpoly
            %       n, m: index sets indicating which channels to keep.
            %       Set to empty to keep all channels in that dimension.
            %
            %   Example:
            %       B = slice(A, 1:2, [1, 3, 5]);
            %
            if isempty(A); B = A; return; end
            [sz1, sz2] = size(A.Mat{1});
            if nargin <= 2; m = 1:sz2; end
            if isempty(n); n = 1:sz1; end
            if isempty(m); m = 1:sz2; end
            m_ = cellfun(@(M) M(n, m), A.Mat, 'UniformOutput', false);
            f_ = cellfun(@(M) M(n, m, :), A.Free, 'UniformOutput', false);
            min_ = cellfun(@(M) M(n, m, :), A.Min, 'UniformOutput', false);
            max_ = cellfun(@(M) M(n, m, :), A.Max, 'UniformOutput', false);
            B = pidpoly(m_, f_, min_, max_);
            B.MustBeMonic_ = A.MustBeMonic_;
        end
        
        function B = mask(A, n, m)
            % MASK Masks filter elements outside indices n and m by setting them to zero and making them non-free.
            %
            %   Syntax:
            %       B = mask(A, n, m)
            %       B = mask(A, n)
            %
            if nargin <= 2; m = A.N; end
            if n <= 0 || m > A.N
                error('Attempting to mask pidpoly outside its range.');
            end
            % Trim ends
            mat_ = A.Mat(1:m);
            free_ = A.Free(1:m);
            min_ = A.Min(1:m);
            max_ = A.Max(1:m);
            % Set to zero
            if n > 1
                mat_{1:n-1} = zeros(size(mat_{1}));
                free_{1:n-1} = false(size(free_{1}));
            end
            B = pidpoly(mat_, free_, min_, max_);
        end
        
        function obj = set.Mat(obj, Mat)
            if ~iscell(Mat)
                Mat = {Mat};
            end
            % Check if monic
            if obj.N > 0 && obj.MustBeMonic_
                if ~issquare(Mat{1}) || ...
                        Mat{1} ~= eye(size(Mat{1}))
                    error('Matrix must be monic.');
                end
            end
            % Validate
            assert(LPVcore.validatepmatrixCellArray(Mat), ...
                ['Attempting to create PIDPOLY object ', ...
                'with PMATRIX objects of different size.']);
            % Convert to pmatrix and check size
            n = -Inf; m = -Inf;
            for i=1:numel(Mat)
                Mat{i} = obj2pmatrix(Mat{i});
                [nTmp, mTmp] = size(Mat{i});
                n = max(n, nTmp); m = max(m, mTmp);
            end
            % Replace zeros by n-by-m pmatrix
            for i=1:numel(Mat)
                if ~isa(Mat{i}, 'pmatrix') && Mat{i} == 0
                    Mat{i} = pzeros(n, m);
                end
            end
            obj.Mat_ = Mat;
            if numel(obj.Mat_) > 0
                [obj.Mat_{:}] = pmatrix.commonrho_(obj.Mat_{:});
            end
        end
        
        function val = get.Mat(obj); val = obj.Mat_; end
        
        function obj = set.Free(obj, Free)
            if ~iscell(Free); Free = {Free}; end
            % Input validation
            for i=1:numel(Free)
                assert(isequal(size(Free{i}, [1, 2, 3]), ...
                    size(obj.Mat{i}, [1, 2, 3])), ...
                    'Each element of ''Free'' must be same size as corresponding pmatrix');
            end
            % Setting monic term free has no effect
            if numel(Free) > 0 && obj.MustBeMonic_
                Free{1} = false(size(Free{1}));
            end
            obj.Free_ = Free;
        end

        % TODO: input validation
        function obj = set.Min(obj, Min)
            obj.Min_ = Min;
        end

        function obj = set.Max(obj, Max)
            obj.Max_ = Max;
        end
        
        function val = get.Free(obj); val = obj.Free_; end
        function val = get.Min(obj); val = obj.Min_; end
        function val = get.Max(obj); val = obj.Max_; end
        
        function F = get.MatNoMonic(obj)
            if obj.N > 1
                if ~ismonic(obj)
                    error('Cannot discard monic term if filter is non-monic.');
                end
                F = obj.Mat_(2:end);
            else
                F = {};
            end
        end
        
        function F = get.FreeNoMonic(obj)
            if obj.N > 1
                F = obj.Free(2:end);
            else
                F = {};
            end
        end
        
        function F = get.MinNoMonic(obj)
            if obj.N > 1
                F = obj.Min(2:end);
            else
                F = {};
            end
        end
        
        function F = get.MaxNoMonic(obj)
            if obj.N > 1
                F = obj.Max(2:end);
            else
                F = {};
            end
        end
        
        function val = ismonic(obj)
            % ISMONIC Returns whether filter is monic, implying Mat{1} = eye.
            val = numel(obj.Mat) >= 1 && issquare(obj) && ...
                (isempty(obj.Mat{1}) || obj.Mat{1} == eye(size(obj.Mat{1})));
        end
        
        function v = isempty(obj)
            v = obj.N == 0;
        end
        
        function v = issquare(obj)
            if isempty(obj); v = true; return; end
            [n, m] = size(obj.Mat{1});
            v = n == m;
        end
        
        function v = isscalar(obj)
            % ISSCALAR Returns true if the polynomial is scalar
            v = size(obj, 1) == 1 && size(obj, 2) == 1;
        end

        function v = isconst(obj)
            % ISCONST Returns whether all coefficients are
            % non-parameter-varying (constant)
            for i=1:obj.N
                if ~isconst(obj.Mat{i})
                    v = false; return;
                end
            end
            v = true;
        end
        
        function v = isct(obj)
            v = strcmp(obj.SchedulingTimeMap.Domain, 'ct');
        end
        
        function v = isdt(obj)
            v = strcmp(obj.SchedulingTimeMap.Domain, 'dt');
        end
        
        function v = iseye(obj)
            % ISEYE Returns whether filter is identity mapping, i.e. I
            v = obj.N == 1 && ismonic(obj);
        end

        function v = isdiag(obj)
            % ISDIAG Returns whether each coefficient is diagonal and all
            % off-diagonal terms are non-free
            v = true;
            for i=1:obj.N
                % Check values
                if ~isdiag(obj.Mat{i})
                    v = false;
                    break;
                end
                % Check free
                f = obj.Free{i};
                for j=1:size(f, 3)
                    if ~isdiag(double(f(:, :, j)))
                        v = false;
                        break
                    end
                end
            end
        end
        
        function pvec = getpvec(filt, ~)
            % GETPVEC Values of parameters in vector form
            %
            %   Syntax:
            %       pvec = getpvec(filt)
            %       pvec = getpvec(filt, 'free')
            %
            %   Inputs:
            %       filt: pidpoly object
            %       'free': whether to only return data for the free
            %           parameters of filt.
            %
            %   Outputs:
            %       pvec: values of the parameters of filt.
            %
            if filt.MustBeMonic_
                pcell = filt.ValueNoMonic;
            else
                pcell = filt.Value;
            end
            ptensor = cell(1, numel(pcell));
            for i=1:numel(pcell)
                [n, ~, ~] = size(pcell{i});
                ptensor{i} = reshape(pcell{i}, n, []);
            end
            pvec = reshape(cell2mat(ptensor), [], 1);      % 2D to 1D
        end
        
        function val = nparams(obj, mode)
            % NPARAMS Returns number of model parameters
            %
            %   Syntax:
            %       np = nparams(obj)
            %       np = nparams(obj, 'free')
            %
            %   Inputs:
            %       return all model parameters unless 'free'.
            %       'free': return free model parameters.
            %
            %   Outputs:
            %       np: number of model parameters.
            %
            %   Note: for monic filters, the first term does not constitute to
            %   the number of parameters.
            %
            if nargin <= 1
                mode = 'all';
            end
            val = 0;
            if strcmp(mode, 'free')
                for i=1:numel(obj.Free)
                    val = val + sum(obj.Free{i}, 'all');
                end
            else
                if obj.MustBeMonic_
                    istart = 2;
                else
                    istart = 1;
                end
                for i=istart:numel(obj.Free)
                    val = val + numel(obj.Free{i});
                end
            end
        end
        
        function val = get.N(obj)
            val = numel(obj.Mat);
        end
        
        function val = get.SchedulingTimeMap(obj)
            if obj.N > 0
                val = obj.Mat{1}.timemap;
            else
                val = timemap;
            end
        end
        
        function obj = set.SchedulingTimeMap(obj, val)
            for i=1:obj.N
                obj.Mat_{i}.timemap = val;
            end
        end
        
        function val = get.Value(obj)
            val = cellfun(@(x) x.matrices, obj.Mat, 'UniformOutput', false);
        end
        
        function obj = set.Value(obj, val)
            for i=1:obj.N
                obj.Mat{i}.matrices = val{i};
            end
        end
        
        function val = get.ValueNoMonic(obj)
            val = cellfun(@(x) x.matrices, obj.Mat(2:end), 'UniformOutput', false);
        end
        
        function obj = set.ValueNoMonic(obj, val)
            for i=2:obj.N
                obj.Mat{i}.matrices = val{i-1};
            end
        end
        
        function B = plusOne(A)
            % PLUSONE Returns A + 1 (add 1 to first filter element).
            %
            %   The parameters of the first filter are automatically set to
            %   non-free.
            %
            %   Syntax:
            %       B = plusOne(A)
            %
            if A.N == 0; error('Cannot do plusOne with empty pidpoly.'); end
            if ~issquare(A); error('Cannot do plusOne with non-square pidpoly.'); end
            M_ = A.Mat_;
            M_{1} = M_{1} + eye(size(M_{1}));
            F_ = A.Free_;
            F_{1} = false(size(F_{1}));
            B = pidpoly(M_, F_, A.Min, A.Max, false);
        end
        
        function B = minusOne(A)
            % MINUSONE Return A - 1 (subtract 1 from first filter element).
            %
            %   For a monic filter, this means that the first filter element is set to 0.
            %   The parameters of the first filter are automatically set to non-free.
            %
            %   Syntax:
            %       B = minusOne(A)
            %
            if A.N == 0; error('Cannot do minusOne with empty pidpoly.'); end
            if ~issquare(A); error('Cannot do minusOne with non-square pidpoly.'); end
            M_ = A.Mat_;
            M_{1} = M_{1} - eye(size(M_{1}));
            F_ = A.Free_;
            F_{1} = false(size(F_{1}));
            B = pidpoly(M_, F_, A.Min, A.Max, false);
        end
        
        function A = zeros(A)
            % ZEROS Set matrices to zero - keeping parameters free.
            %
            %   Syntax:
            %       B = zeros(A)
            %
            if A.N == 0; return; end
            M_ = A.Mat_;
            for i=1:A.N
                M_{i}.matrices = zeros(size(M_{i}.matrices));
            end
            A.Mat_ = M_;
        end
        
        function A = rand(A, interval)
            % RAND Set free coefficients to values drawn from uniform random
            % distribution.
            %
            %   Syntax:
            %       B = rand(A)
            %       B = rand(A, interval)
            %
            %   Inputs:
            %       A: pidpoly object
            %       interval: vector of two doubles denoting interval from which random values are drawn.
            %           Default: [0, 1].
            %
            narginchk(1, 2);
            if nargin <= 1
                interval = [0, 1];
            end
            assert(isvector(interval) && isnumeric(interval) && ...
                isreal(interval) && numel(interval) == 2 && ...
                interval(2) >= interval(1), '''interval'' must be a vector of 2 doubles');
            
            if A.N == 0; return; end
            M_ = A.Mat_;
            for i=1:A.N
                newValues = interval(1) + (interval(2) - interval(1)) .* rand(size(M_{i}.matrices));
                M_{i}.matrices(A.Free{i}) = newValues(A.Free{i});
            end
            A.Mat_ = M_;
        end
        
        function obj = defaultStruct_(obj, Monic)
            %DEFAULTSTRUCT Populate structure with defaults for Free, Min, Max and Value
            free = cell(1, obj.N);
            min = cell(1, obj.N);
            max = cell(1, obj.N);
            for i=1:obj.N
                pmat = obj.Mat{i};
                [n, m, o] = size(pmat);
                if (obj.ZeroIsNonFree && o == 1 && (isempty(pmat) || pmat == 0) || ...
                        (Monic && i == 1))
                    free{i} = false(n, m, 1);
                else
                    if obj.ZeroIsNonFree
                        free{i} = pmat.matrices ~= 0;
                    else
                        free{i} = true(n, m, o);
                    end
                end
                min{i} = -Inf(n, m, o);
                max{i} = Inf(n, m, o);
            end
            obj.Free = free; obj.Min = min; obj.Max = max;
        end
        
        function B = freeze(obj, p, mode)
        % FREEZEPMATCALL_ Freeze pidpoly and convert to
        % representation suitable for IDPOLY.
        %
        %   Syntax:
        %       B = freeze(Mat, p)
        %       B = freeze(Mat, p, 'diag')
        %       B = freeze(Mat, p, 'F')
        %
        %   Inputs:
        %       A: cell array of pmatrix objects
        %       p: value of frozen scheduling parameter
        %           'diag': diagonal elements are extracted and placed in
        %               N-by-1 cell array of row vectors. Used for C and D.
        %           'F': special scheme for F-polynomial. In lpvidpoly, F
        %               represents a matrix polynomial which is inverted. In
        %               idpoly, F represents a collection of scalar polynomials
        %               (which are also element-wise inverted). 
        %
        %   Outputs:
        %       B: n-by-m cell array of row vectors, or row vector if n == m ==
        %       1.
        %
        narginchk(2, 3);
        if nargin <= 2
            mode = '';
        end
        assert(any(strcmp(mode, {'', 'diag', 'F'})), ...
            '''mode'' must be ''diag'' or ''F'', not %s', mode);
        B = [];
        if obj.N == 0; return; end
        % Freeze pmatrix objects
        T = cell(1, obj.N);
        for i=1:obj.N
            T{i} = freeze(obj.Mat{i}, p);
        end

        % Convert cell array of matrices to cell of row vectors
        [n, m] = size(obj.Mat{1});
        assert(isempty(mode) || n == m, ...
            '''%s'' can only be used for square polynomials', mode);
        o = numel(T);
        if strcmp(mode, 'diag')
            B = cell(n, 1);
        else
            B = cell(n, m);
        end
        for i=1:n
            for j=1:m
                coeff = NaN(1, o);
                for k=1:o
                    T_ = T{k};
                    coeff(k) = T_(i, j);
                end
                if strcmp(mode, '')
                    B{i, j} = coeff;
                elseif strcmp(mode, 'F')
                    % Coefficients are all row vectors
                    % Each element should be repeated along the rows, 
                    % E.g. [1, 2] --> {[1, 2], [1, 2]}
                    if i == j
                        % Repeat along all rows
                        for k=1:m
                            B{i, k} = coeff;
                        end
                    else
                        assert(all(coeff == 0), 'Coefficients of ''F'' must be diagonal');
                    end
                elseif strcmp(mode, 'diag')
                    if i == j
                        B{i} = coeff;
                    else
                        assert(all(coeff == 0), 'Off-diagonal entries must be 0 if ''diag'' is passed');
                    end
                end
            end
        end
        end
        
        function obj = uplus(obj); end
        function A = uminus(A)
            for i=1:A.N
                A.Mat{i} = -A.Mat{i};
                tmp = A.Min{i};
                A.Min{i} = -A.Max{i};
                A.Max{i} = -tmp;
            end
        end
        
        function C = vertcat(varargin); C = cat(1, varargin{:}); end
        function C = horzcat(varargin); C = cat(2, varargin{:}); end
        
        function C = cat(dim, varargin)
            [varargin{1:nargin-1}] = pidpoly.commonrho_(varargin{:});
            C = varargin{1};
            
            % iteratively concatenate pairs of pidpoly objects
            for i=2:nargin-1
                C = cat_(dim, C, varargin{i});
            end
        end
        
        function sz = size(obj, varargin)
            if numel(obj.Mat) > 0
                sz = size(obj.Mat{1}, varargin{:});
            else
                sz = [0, 0];
            end
        end
    end
    
    methods (Hidden, Static)
        function [varargout] = commonrho_(varargin)
        %COMMONRHO_ Reformulates input pidpoly objects to share common ext.
        %scheduling parameter.
        %
        %   Syntax:
        %       [B1, ..., Bn] = commonrho_(A1, ..., An)
        %       [B1, ..., Bn, tm] = commonrho_(A1, ..., An)
        %
        %   Inputs:
        %       A1, ..., An: pidpoly objects.
        %
        %   Outputs:
        %       B1, ..., Bn: pidpoly objects sharing same timemap.
        %       tm: timemap
        %

            if nargin ~= nargout && nargin + 1 ~= nargout
                error('Number of output arg. must be number of input args + {1/0}.');
            end

            tm = cell(1, nargin);
            for i=1:nargin
                tm{i} = varargin{i}.SchedulingTimeMap;
            end

            tmCommon = tm{1};
            for i=2:nargin
                tmCommon = union(tmCommon, tm{i});
            end

            for i=1:nargin
                varargin{i}.SchedulingTimeMap = tmCommon;
            end

            if nargout == nargin
                varargout = varargin;
            else
                varargout(1:nargin) = varargin;
                varargout{nargout} = tmCommon;
            end

        end
    end
    
    methods (Hidden)
        function C = blkdiag_(A, B)
            %BLKDIAG_ Block-diagonal concatenation of 2 pidpoly objects
            %with common ext. scheduling parameter
            % 
            % TODO: add BLKDIAG method for arbitrary nr. of pidpoly
            % objects
            %
            %   C = [A, 0; <-- R1
            %        0, B]
            %
            R1 = cat(2, A, pidpoly(zeros(size(A, 1), size(B, 2))));
            R2 = cat(2, pidpoly(zeros(size(B, 1), size(A, 2))), B);
            C = cat(1, R1, R2);
        end

        function C = cat_(dim, A, B)
            %CAT_ Concatenates 2 pidpoly objects with common ext.
            %scheduling parameter.
            %
            %   Synatx:
            %       C = cat_(dim, A, B)
            %
            %   Inputs:
            %       dim (int): dimension along which to concatenate
            %       A, B: pidpoly objects to concatenate. Assumptions:
            %           * same ext. scheduling variable (commonrho)
            %
            %   Outputs:
            %       C: concatenated pidpoly object
            %
            if isempty(A)
                C = B;
                return;
            end
            if isempty(B)
                C = A;
                return;
            end
            % Make same number of filters
            NA = A.N;
            NB = B.N;
            if NA > NB
                B = extend_(B, NA);
            elseif NB > NA
                A = extend_(A, NB);
            end
            % From here, A and B have same NMat
            CFilt = cell(1, A.N);
            CFree = CFilt; CMin = CFilt; CMax = CFilt;
            for i=1:A.N
                [CFilt{i}, IA, IB] = cat(dim, A.Mat{i}, B.Mat{i});
                o = size(CFilt{i}, 3);
                % Pad parameter properties to o
                CFree{i} = cat(dim, reidxpad_(A.Free{i}, IA, o, false), ...
                    reidxpad_(B.Free{i}, IB, o, false));
                CMin{i} = cat(dim, reidxpad_(A.Min{i}, IA, o, -Inf), ...
                    reidxpad_(B.Min{i}, IB, o, -Inf));
                CMax{i} = cat(dim, reidxpad_(A.Max{i}, IA, o, +Inf), ...
                    reidxpad_(B.Max{i}, IB, o, +Inf));
            end
            C = pidpoly(CFilt, CFree, CMin, CMax);
        end
        
        function A = extend_(A, NewN)
            % EXTEND_ Extends pidpoly to n elements
            %
            %   Newly created elements have the following parameter
            %   settings:
            %
            %       Free: false, Min: -Inf, Max: +Inf, Value: 0
            %
            if A.N == 0; error('Trying to extend empty pidpoly.'); end
            [n, m] = size(A.Mat{1});
            for i=A.N+1:NewN
                A.Mat{i} = zeros(n, m);
                A.Free{i} = false(n, m);
                A.Min{i} = -Inf(n, m);
                A.Max{i} = Inf(n, m);
            end
        end
    end
end

function B = reidxpad_(A, I, sz, value)
    % REIDXPAD_ Apply padding and reindexing in third dimension of
    % tensor.
    %
    %   Syntax:
    %       B = REIDXPAD_(A, I, size, value)
    %
    %   Inputs:
    %       A: tensor
    %       I: reindexaction over 3rd dimension
    %       size: size to pad
    %       value: value to pad
    %
    %   Outputs:
    %       B: matrices padded to same size in dimension dim with value.
    %
    %   Used in pidpoly/cat_.
    %
    [n, m, ~] = size(A);
    B = NaN(n, m, sz);
    % pad
    for i=1:sz
        B(:, :, i) = value;
    end
    if islogical(value)
        B = logical(B);
    end
    % re-index
    for k=1:numel(I)
        i = I(k);
        B(:, :, i) = A(:, :, k);
    end
end

