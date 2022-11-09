classdef lpvidfilt
    %LPVIDFILT Helper class for lpvidpoly
    %
    %   An LPVIDPOLY system consists of 5 filters: A, B, C, D and F.
    %   These filters are parameter-varying collections of PMATRIX objects
    %   that each represent a different time-delay or differentiation,
    %   i.e.:
    %
    %       A(p, q) := A0(p) + A1(p) q^{-1} + ... + AN(p) q^{-N}
    %
    %   This filter structure is captured by LPVIDFILT. It provides
    %   convenient methods for concatenation, retrieving information on the
    %   parameter set, and some operations such as subtraction of 1 (A -
    %   1).
    %
    %   You generally don't need to manipulate LPVIDFILT objects directly:
    %   the logic is handled by the higher-level LPVIDPOLY object.
    %
    %   Syntax:
    %       A = lpvidfilt(Mat, Free, Min, Max, monic, Name, Value)
    %
    %   Inputs:
    %       Mat: cell array of PMATRIX objects A0, ..., AN
    %       Free: cell array of logicals indicating which parameters in the
    %       A0, ..., AN are free. [] creates the default.
    %       Min, Max: cell array of doubles indicating the range of the
    %       free parameters. [] creates the default (-Inf to +Inf).
    %       monic: logical indicating whether the object should be monic.
    %
    %   Name, Value pairs:
    %       'ZeroIsNonFree': logical indicating whether a 0 entry in Mat
    %       constitutes a non-free parameter  (e.g. {A0, 0, A2}). Default:
    %       true.
    %
    
    properties
        Min     % Cell array of min. values
        Max     % Cell array of max. values
        ZeroIsNonFree = true   % Boolean indicating whether zero entries in A are non-free parameters. Default: true
    end
    
    properties (Dependent)
        Mat
        Free
        N               % Number of filters
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
        MustBeMonic_
    end
    
    methods
        function obj = lpvidfilt(Mat, Free, Min, Max, MustBeMonic, ZeroIsNonFree)
            %LPVIDFILT Construct an instance of this class
            %
            %   Syntax:
            %
            %       F = lpvidfilt(Mat, Free, Min, Max)
            %       F = lpvidfilt(Mat, [], [], [], monic, ZeroIsNonFree)
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
            %       A: lpvidfilt object
            %
            %   Outputs:
            %       D: cell array of lpvidfilt objects representing the partial derivative
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
                                D{DCount} = lpvidfilt([Z(:)', {P}]);
                                DCount = DCount + 1;
                            end
                        end
                    end
                end
                Z = [Z{:}, {zeros(n, m)}];
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
            %       A: lpvidfilt
            %       n: number of terms to keep
            %
            M_ = A.Mat_(1:n);
            F_ = A.Free_(1:n);
            Min_ = A.Min(1:n);
            Max_ = A.Max(1:n);
            B = lpvidfilt(M_, F_, Min_, Max_);
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
            %       A: lpvidfilt
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
            M_ = cellfun(@(M) M(n, m), A.Mat, 'UniformOutput', false);
            F_ = cellfun(@(M) M(n, m), A.Free, 'UniformOutput', false);
            Min_ = cellfun(@(M) M(n, m), A.Min, 'UniformOutput', false);
            Max_ = cellfun(@(M) M(n, m), A.Max, 'UniformOutput', false);
            B = lpvidfilt(M_, F_, Min_, Max_);
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
                error('Attempting to mask lpvidfilt outside its range.');
            end
            % Trim ends
            M_ = A.Mat(1:m);
            F_ = A.Free(1:m);
            Min_ = A.Min(1:m);
            Max_ = A.Max(1:m);
            % Set to zero
            if n > 1
                M_{1:n-1} = zeros(size(M_{1}));
                F_{1:n-1} = false(size(F_{1}));
            end
            B = lpvidfilt(M_, F_, Min_, Max_);
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
                ['Attempting to create LPVIDFILT object ', ...
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
            % Setting monic term free has no effect
            if numel(Free) > 0 && obj.MustBeMonic_
                Free{1} = false(size(Free{1}));
            end
            obj.Free_ = Free;
        end
        
        function val = get.Free(obj); val = obj.Free_; end
        
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
        
        function pvec = getpvec(filt, ~)
            % GETPVEC Values of parameters in vector form
            %
            %   Syntax:
            %       pvec = getpvec(filt)
            %       pvec = getpvec(filt, 'free')
            %
            %   Inputs:
            %       filt: lpvidfilt object
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
            if A.N == 0; error('Cannot do plusOne with empty lpvidfilt.'); end
            if ~issquare(A); error('Cannot do plusOne with non-square lpvidfilt.'); end
            M_ = A.Mat_;
            M_{1} = M_{1} + eye(size(M_{1}));
            F_ = A.Free_;
            F_{1} = false(size(F_{1}));
            B = lpvidfilt(M_, F_, A.Min, A.Max, false);
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
            if A.N == 0; error('Cannot do minusOne with empty lpvidfilt.'); end
            if ~issquare(A); error('Cannot do minusOne with non-square lpvidfilt.'); end
            M_ = A.Mat_;
            M_{1} = M_{1} - eye(size(M_{1}));
            F_ = A.Free_;
            F_{1} = false(size(F_{1}));
            B = lpvidfilt(M_, F_, A.Min, A.Max, false);
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
            %       A: lpvidfilt object
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
            obj.Free = cell(1, obj.N);
            obj.Min = cell(1, obj.N);
            obj.Max = cell(1, obj.N);
            for i=1:obj.N
                pmat = obj.Mat{i};
                [n, m, o] = size(pmat);
                if (obj.ZeroIsNonFree && o == 1 && (isempty(pmat) || pmat == 0) || ...
                        (Monic && i == 1))
                    obj.Free{i} = false(n, m, 1);
                else
                    if obj.ZeroIsNonFree
                        obj.Free{i} = pmat.matrices ~= 0;
                    else
                        obj.Free{i} = true(n, m, o);
                    end
                end
                obj.Min{i} = -Inf(n, m, o);
                obj.Max{i} = Inf(n, m, o);
            end
        end
        
        function B = freeze(obj, p)
        % FREEZEPMATCALL_ Freeze lpvidfilt and convert to
        % representation suitable for IDPOLY.
        %
        %   Syntax:
        %       B = freeze(Mat, p)
        %
        %   Inputs:
        %       A: cell array of pmatrix objects
        %       p: value of frozen scheduling parameter
        %
        %   Outputs:
        %       B: n-by-m cell array of row vectors, or row vector if n == m ==
        %       1.
        %
        B = [];
        if obj.N == 0; return; end

        % Freeze pmatrix objects
        T = cell(1, obj.N);
        for i=1:obj.N
            T{i} = freeze(obj.Mat{i}, p);
        end

        % Convert cell array of matrices to cell of row vectors
        [n, m] = size(obj.Mat{1});
        o = numel(T);
        B = cell(n, m);
        for i=1:n
            for j=1:m
                coeff = NaN(1, o);
                for k=1:o
                    T_ = T{k};
                    coeff(k) = T_(i, j);
                end
                B{i, j} = coeff;
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
            [varargin{1:nargin-1}] = lpvidfilt.commonrho_(varargin{:});
            C = varargin{1};
            
            % iteratively concatenate pairs of pmatrix objects
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
        %COMMONRHO_ Reformulates input lpvidfilt objects to share common ext.
        %scheduling parameter.
        %
        %   Syntax:
        %       [B1, ..., Bn] = commonrho_(A1, ..., An)
        %       [B1, ..., Bn, tm] = commonrho_(A1, ..., An)
        %
        %   Inputs:
        %       A1, ..., An: lpvidfilt objects.
        %
        %   Outputs:
        %       B1, ..., Bn: lpvidfilt objects sharing same timemap.
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
    
    methods (Access = 'private')
        function C = cat_(dim, A, B)
            %CAT_ Concatenates 2 lpvidfilt objects with common ext.
            %scheduling parameter.
            %
            %   Synatx:
            %       C = cat_(dim, A, B)
            %
            %   Inputs:
            %       dim (int): dimension along which to concatenate
            %       A, B: lpvidfilt objects to concatenate. Assumptions:
            %           * same ext. scheduling variable (commonrho)
            %
            %   Outputs:
            %       C: concatenated lpvidfilt object
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
                % Padd parameter properties to o
                CFree{i} = cat(dim, reidxpad_(A.Free{i}, IA, o, false), ...
                    reidxpad_(B.Free{i}, IB, o, false));
                CMin{i} = cat(dim, reidxpad_(A.Min{i}, IA, o, -Inf), ...
                    reidxpad_(B.Min{i}, IB, o, -Inf));
                CMax{i} = cat(dim, reidxpad_(A.Max{i}, IA, o, +Inf), ...
                    reidxpad_(B.Max{i}, IB, o, +Inf));
            end
            C = lpvidfilt(CFilt, CFree, CMin, CMax);
        end
        
        function A = extend_(A, NewN)
            % EXTEND_ Extends lpvidfilt to n elements
            %
            %   Newly created elements have the following parameter
            %   settings:
            %
            %       Free: false, Min: -Inf, Max: +Inf, Value: 0
            %
            if A.N == 0; error('Trying to extend empty lpvidfilt.'); end
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
    %   Used in lpvidfilt/cat_.
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

