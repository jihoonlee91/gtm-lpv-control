classdef lpvidpoly < lpvrep & lpvidsys
    %LPVIDPOLY Polynomial LPV model with identifiable parameters
    %
    %   Represents the following model structure:
    %
    %       A(q, p) yhat = y + v
    %       F(q, p) y    = B(q, p) u
    %       D(q, p) v    = C(q, p) e
    %
    %       with:
    %
    %           A(q, p): eye(ny) + A1(p) q^{-1} + ... + A_na(p) q^{-na}
    %           B(q, p): B0(p) q^{-1} + ... + B_nb(p) q^{-nb}
    %           C, D, F: similar as A
    %           yhat: measured output
    %           y: noise-free output
    %           u: input
    %           v: colored noise
    %           e: white noise
    %
    %   Syntax:
    %       sys = lpvidpoly(A, B, C, D, F, NoiseVariance, Ts)
    %       sys = lpvidpoly(A, B, C, D, F, NoiseVariance, Ts, Name, Value)
    %       sys = lpvidpoly(A)
    %       sys = lpvidpoly(A, [], C, D, [], NoiseVariance, Ts)
    %       sys = lpvidpoly(sys0)
    %
    %   Inputs:
    %       A, B, C, D, F: pmatrix or cell array of pmatrix objects.
    %       NoiseVariance: covariance matrix of model innovation noise
    %       Ts: sampling time. CT: Ts = 0. Unspecified DT: Ts = -1.
    %
    %   Name-Value Pair Arguments:
    %       ZeroIsNonFree: logical indicating whether 0s indicate that the
    %       corresponding variables are non-free. E.g. lpvidpoly({0, a2})
    %       creates a model with the first term of A non-free. Default:
    %       true.
    %
    
    properties
        % Noise covariance specified as an Ny-by-Ny positive semidefinite
        % matrix, where Ny is the number of outputs. Default: eye(Ny).
        NoiseVariance
        Covariance          % Parameter covariance matrix
        Type                % System type: 'ARX', 'ARMAX', 'OE' or 'BJ'
    end
    
    properties (SetAccess = 'private')
        ZeroIsNonFree       % Logical indicating whether 0s are non-free parameters (default: true)
    end
    
    properties (Dependent)
        A               % Monic pidpoly
        B               % pidpoly
        C               % Monic pidpoly
        D               % Monic pidpoly
        F               % Monic pidpoly
        Na              % Maximum delay of A
        Nb              % Maximum delay of B
        Nc              % Maximum delay of C
        Nd              % Maximum delay of D
        Nf              % Maximum delay of F
    end
    
    properties (Access = private)
        A_ = {}
        B_ = {}
        C_ = {}
        D_ = {}
        F_ = {}
    end
    
    methods
        function obj = lpvidpoly(varargin)
            %LPVIDPOLY Construct an instance of this class
            %
            %   See class description.
            %
            isValidFilter = @(x) isa(x, 'pidpoly') ...
                || isa(x, 'pmatrix') || isa(x, 'numeric') ...
                || (iscell(x) && all(cellfun(@(f) (isa(f, 'pmatrix') || isnumeric(f)), x), 'all'));
            
            p = inputParser();
            p.KeepUnmatched = true;
            addOptional(p, 'A', {}, @(x) isValidFilter(x));
            addOptional(p, 'B', {}, @(x) isValidFilter(x));
            addOptional(p, 'C', {}, @(x) isValidFilter(x));
            addOptional(p, 'D', {}, @(x) isValidFilter(x));
            addOptional(p, 'F', {}, @(x) isValidFilter(x));
            addOptional(p, 'NoiseVariance', 1, @(x) isnumeric(x) || isempty(x));
            addOptional(p, 'Ts', [], @(x) isscalar(x) || isempty(x));
            addParameter(p, 'ZeroIsNonFree', true, @(x) islogical(x));
            parse(p, varargin{:});
            
            % Cast input to pidpoly object
            obj.ZeroIsNonFree = p.Results.ZeroIsNonFree;
            A = makeFiltOrEmpty_(p.Results.A, true, obj.ZeroIsNonFree);
            B = makeFiltOrEmpty_(p.Results.B, false, obj.ZeroIsNonFree);
            C = makeFiltOrEmpty_(p.Results.C, true, obj.ZeroIsNonFree);
            D = makeFiltOrEmpty_(p.Results.D, true, obj.ZeroIsNonFree);
            F = makeFiltOrEmpty_(p.Results.F, true, obj.ZeroIsNonFree);
            
            % Get output dimension
            if ~isempty(A)
                ny = size(A.Mat{1}, 1);
            elseif ~isempty(B)
                ny = size(B.Mat{1}, 1);
            elseif ~isempty(C)
                ny = size(C.Mat{1}, 1);
            elseif ~isempty(D)
                ny = size(D.Mat{1}, 1);
            elseif ~isempty(F)
                ny = size(F.Mat{1}, 1);
            end

            % Convert empty filters to identity
            if isempty(A); A = pidpoly({eye(ny)}, [], [], [], true); end
            if isempty(B); B = pidpoly({[]}, [], [], [], false); end
            if isempty(C); C = pidpoly({eye(ny)}, [], [], [], true); end
            if isempty(D); D = pidpoly({eye(ny)}, [], [], [], true); end
            if isempty(F); F = pidpoly({eye(ny)}, [], [], [], true); end

            obj.A_ = A; obj.B_ = B; obj.C_ = C; obj.D_ = D; obj.F_ = F;
            obj.A = A; obj.B = B; obj.C = C; obj.D = D; obj.F = F;
            
            % Set NoiseVariance and convert to matrix if needed
            NoiseVariance = p.Results.NoiseVariance;
            if isempty(NoiseVariance); NoiseVariance = 1; end
            if isscalar(NoiseVariance)
                NoiseVariance = eye(obj.Ny) * NoiseVariance; 
            end
            obj.NoiseVariance = NoiseVariance;
            obj = parsePropertyValuePairs(obj, p.Unmatched);
            
            % Check sampling time
            Ts = p.Results.Ts;
            if isempty(Ts)
                if strcmp(obj.SchedulingTimeMap.Domain, 'ct')
                    Ts = 0;
                else
                    Ts = -1;
                end
            end
            if strcmp(obj.SchedulingTimeMap.Domain, 'ct') && Ts ~= 0
                error('Ts must be 0 for CT model.');
            elseif strcmp(obj.SchedulingTimeMap.Domain, 'dt') && Ts == 0
                error('Ts must be a positive scalar, or -1, for DT model.');
            elseif strcmp(obj.SchedulingTimeMap.Domain, 'undefined')
                if Ts == 0
                    obj.SchedulingTimeMap.Domain = 'ct';
                else
                    obj.SchedulingTimeMap.Domain = 'dt';
                end
            end
            obj.Ts_ = Ts;
        end
        
        function val = nparams(obj, f)
            % NPARAMS Number of model parameters.
            %
            %   Syntax:
            %       np = nparams(sys)
            %       np = nparams(sys, 'free')
            %
            %   Inputs:
            %       sys: lpvidpoly object
            %       'free': returns number of free model parameters
            %
            %   Ouputs:
            %       np: number of (free) model parameters. Note that the
            %       leading coefficients of A, D and F are not counted as
            %       model parameters.
            %
            if nargin <= 1
                f = 'all';
            end
            val = nparams(obj.A_, f);
            val = val + nparams(obj.B_, f);
            val = val + nparams(obj.C_, f);
            val = val + nparams(obj.D_, f);
            val = val + nparams(obj.F_, f);
        end
        
        function val = isfree(obj)
            % ISFREE Returns logical vector indicating which parameters are optimizable.
            val = filt2vec_([obj.A.FreeNoMonic, obj.B.Free, obj.C.FreeNoMonic, ...
                obj.D.FreeNoMonic, obj.F.FreeNoMonic]);
        end
        
        function [min, max] = getpMinMax(obj)
            % GETPMINMAX Min. and max. values of parameters in vector form
            %
            %   Syntax:
            %       [min, max] = getpMinMax(obj)
            %
            %   Outputs:
            %       min: nparams-by-1 vector of min. values
            %       max: nparams-by-1 vector of max. values
            %
            min = filt2vec_([obj.A.MinNoMonic, obj.B.Min, obj.C.MinNoMonic, ...
                obj.D.MinNoMonic, obj.F.MinNoMonic]);
            max = filt2vec_([obj.A.MaxNoMonic, obj.B.Max, obj.C.MaxNoMonic, ...
                obj.D.MaxNoMonic, obj.F.MaxNoMonic]);
        end
        
        function pvec = getpvec(obj, f)
            % GETPVEC Values of parameters in vector form
            %
            %   Syntax:
            %       pvec = getpvec(sys)
            %       pvec = getpvec(sys, 'free')
            %
            %   Inputs:
            %       sys: lpvidpoly object
            %       'free': whether to only return data for the free
            %           parameters of sys.
            %
            %   Outputs:
            %       pvec: values of the parameters of sys.
            %
            if nargin <= 1
                f = '';
            end
            if strcmpi(f, 'free')
                error('GETPVEC with "free" is not yet implemented.');
            end
            pvecA = getpvec(obj.A, f);
            pvecB = getpvec(obj.B, f);
            pvecC = getpvec(obj.C, f);
            pvecD = getpvec(obj.D, f);
            pvecF = getpvec(obj.F, f);
            pvec = [pvecA; pvecB; pvecC; pvecD; pvecF];
        end

        function obj = setpvec(obj, p)
            % SETPVEC Set values of free parameters in vector form
            %
            %   Syntax:
            %       obj = setpvec(obj, delta)
            %
            %   Inputs:
            %       p: nparams(obj, 'free')-by-1 vector of parameter values
            %
            %   See also APPLYDELTAP

            % Input validation
            narginchk(2, 2);
            assert(isnumeric(p) && iscolumn(p) && isreal(p) && ...
                numel(p) == nparams(obj, 'free'), ...
                '''p'' must be a column vector with size the number of free parameters');

            % Loop through filter elements
            [obj.A_, p] = setpvec(obj.A_, p);
            [obj.B_, p] = setpvec(obj.B_, p);
            [obj.C_, p] = setpvec(obj.C_, p);
            [obj.D_, p] = setpvec(obj.D_, p);
            [obj.F_, ~] = setpvec(obj.F_, p);
        end
        
        function obj = applydeltap(obj, delta)
            % APPLYDELTAP Update values of free parameters in vector form
            %
            %   Syntax:
            %       obj = applydeltap(obj, delta)
            %
            %   Inputs:
            %       delta: nparams(obj, 'free')-by-1 vector of delta values
            %       that are added to the current value of the free
            %       parameter.
            %
            
            % Input validation
            if numel(delta) < nparams(obj, 'free')
                error('Too few parameters were provided to update values.');
            end
            
            % Loop through filter elements
            [obj.A_, delta] = applydeltap(obj.A_, delta);
            [obj.B_, delta] = applydeltap(obj.B_, delta);
            [obj.C_, delta] = applydeltap(obj.C_, delta);
            [obj.D_, delta] = applydeltap(obj.D_, delta);
            [obj.F_, ~] = applydeltap(obj.F_, delta);
        end
        
        function val = get.A(obj); val = obj.A_; end
        function val = get.B(obj); val = obj.B_; end
        function val = get.C(obj); val = obj.C_; end
        function val = get.D(obj); val = obj.D_; end
        function val = get.F(obj); val = obj.F_; end
        
        function obj = set.A(obj, A)
            if ~isa(A, 'pidpoly'); A = pidpoly(A, [], [], [], true, obj.ZeroIsNonFree); end
            if ~ismonic(A); error('A must be monic'); end
            obj.A_ = A;
            obj = updateFilt_(obj);
        end
        
        function obj = set.B(obj, B)
            if ~isa(B, 'pidpoly'); B = pidpoly(B, [], [], [], false, obj.ZeroIsNonFree); end
            obj.B_ = B;
            obj = updateFilt_(obj);
        end
        
        function obj = set.C(obj, C)
            if ~isa(C, 'pidpoly'); C = pidpoly(C, [], [], [], true, obj.ZeroIsNonFree); end
            if ~ismonic(C); error('C must be monic'); end
            obj.C_ = C;
            obj = updateFilt_(obj);
        end
        
        function obj = set.D(obj, D)
            if ~isa(D, 'pidpoly'); D = pidpoly(D, [], [], [], true, obj.ZeroIsNonFree); end
            if ~ismonic(D); error('D must be monic'); end
            obj.D_ = D;
            obj = updateFilt_(obj);
        end
        
        function obj = set.F(obj, F)
            if ~isa(F, 'pidpoly'); F = pidpoly(F, [], [], [], true, obj.ZeroIsNonFree); end
            if ~ismonic(F); error('F must be monic'); end
            obj.F_ = F;
            obj = updateFilt_(obj);
        end
        
        function val = getNu(obj)
            if ~isempty(obj.B)
                val = size(obj.B_.Mat{1}, 2);
            else
                val = 0;
            end
        end
        
        function val = getNy(obj)
            if obj.Na >= 0; val = size(obj.A_.Mat{1}, 1);
            elseif obj.Nb >= 0; val = size(obj.B_.Mat{1}, 1);
            elseif obj.Nc >= 0; val = size(obj.C_.Mat{1}, 1);
            elseif obj.Nd >= 0; val = size(obj.D_.Mat{1}, 1);
            elseif obj.Nf >= 0; val = size(obj.F_.Mat{1}, 1); 
            end
        end
        
        function val = get.Na(obj)
            val = obj.A_.N - 1;
        end
        
        function val = get.Nb(obj)
            val = obj.B_.N - 1;
        end
        
        function val = get.Nc(obj)
            val = obj.C_.N - 1;
        end
        
        function val = get.Nd(obj)
            val = obj.D_.N - 1;
        end
        
        function val = get.Nf(obj)
            val = obj.F_.N - 1;
        end

        function tf = isar(obj)
            % ISAR Returns logical 1 (true) if object represents LPV-AR
            % system (Nu = 0 and C, D == eye)
            tf = obj.Nu == 0 && iseye(obj.C) && iseye(obj.D);
        end
        
        function tf = isarx(obj)
            % ISARX Returns logical 1 (true) if object represents LPV-ARX
            % system (C, D, F == eye)
            tf = obj.Nu > 0 && iseye(obj.C) && iseye(obj.D) && iseye(obj.F);
        end
        
        function tf = isarmax(obj)
            % ISARMAX Returns logical 1 (true) if object represents
            % LPV-ARMAX system (D, F == eye)
            tf = obj.Nu > 0 && iseye(obj.D) && iseye(obj.F);
        end
        
        function tf = isoe(obj)
            % ISOE Returns logical 1 (true) if object represents LPV-OE
            % system (A, C, D == eye)
            tf = obj.Nu > 0 && iseye(obj.A) && iseye(obj.C) && iseye(obj.D);
        end
        
        function tf = isbj(obj)
            % ISBJ Returns logical 1 (true) if object represents LPV-BJ
            % system (A == eye)
            tf = obj.Nu > 0 && iseye(obj.A);
        end
        
        function name = modelStructureName(obj)
            % MODELSTRUCTURENAME Return name of model structure ('ARX',
            % 'ARMAX', 'OE' or 'BJ'
            %
            %   Syntax:
            %       name = modelStructureName(obj)
            %
            if isar(obj); name = 'AR'; return; end
            if isarx(obj); name = 'ARX'; return; end
            if isarmax(obj); name = 'ARMAX'; return; end
            if isoe(obj); name = 'OE'; return; end
            if isbj(obj); name = 'BJ'; return; end
            name = 'IDPOLY';
        end
        
        function val = get.Type(obj); val = modelStructureName(obj); end

        function varargout = subsref(obj, S)
            %SUBSREF Subscripted referencing for lpvidpoly objects.
            %
            %   F = P(i:j, k:l)
            %
            %   Inputs:
            %       P: indexed lpvidpoly object.
            %       S: indexing structure.
            %
            %   Outputs:
            %       F: result of indexing expression.
            %
            %   This method overloads the builtin SUBSREF functionality for the case of
            %   2-dimensional parentheses indexing. In this case, it indexes the
            %   output and input channels of the system.
            %
            
            % Only subsref of the following form are overloaded:
            %
            %   indexing: P(x, y)
            %
            
            if numel(S) == 1 && strcmp(S.type, '()') && numel(S.subs) <= 2 ...
                    && ~(numel(S.subs) == 1 && isa(S.subs{1}, 'char'))  % ':'
                output_channels = S.subs{1};
                input_channels = S.subs{2};
                % Two subreference structures are needed: So (for Ny-by-Ny
                % polynomials A, C, D and F) and Si (for Ny-by-Nu
                % polynomial B)
                So = S;
                So.subs = {output_channels, output_channels};
                Si = S;
                Si.subs = {output_channels, input_channels};
                % Apply subreferencing to polynomials
                obj.A_ = subsref(obj.A_, So);
                obj.B_ = subsref(obj.B_, Si);
                obj.C_ = subsref(obj.C_, So);
                obj.D_ = subsref(obj.D_, So);
                obj.F_ = subsref(obj.F_, So);
                % Apply subreference to properties
                obj.NoiseVariance = obj.NoiseVariance(output_channels, output_channels);
                obj.InputName = obj.InputName(input_channels);
                obj.InputUnit = obj.InputUnit(input_channels);
                obj.InputDelay = obj.InputDelay(input_channels);
                obj.OutputName = obj.OutputName(output_channels);
                obj.OutputUnit = obj.OutputUnit(output_channels);
                obj.Covariance = [];
                % TODO: input and output groups
                varargout{1} = obj;
            else
                [varargout{1:nargout}] = builtin('subsref', obj, S);
            end
        end

        function sys = vertcat(sys1, sys2)
            sys = sys1;
            sys.A_ = blkdiag_(sys1.A_, sys2.A_);
            sys.B_ = vertcat(sys1.B_, sys2.B_);
            sys.C_ = blkdiag_(sys1.C_, sys2.C_);
            sys.D_ = blkdiag_(sys1.D_, sys2.D_);
            sys.F_ = blkdiag_(sys1.F_, sys2.F_);
            sys.NoiseVariance = blkdiag(sys1.NoiseVariance, sys2.NoiseVariance);
            sys.OutputName = [sys1.OutputName; sys2.OutputName];
            sys.OutputUnit = [sys1.OutputUnit; sys2.OutputUnit];
        end
    end
    
    methods (Access = 'private')
    
        function obj = updateFilt_(obj)
            % Get common ext. scheduling variable and timemap
            [obj.A_, obj.B_, obj.C_, obj.D_, obj.F_, obj.SchedulingTimeMap] = ...
                pidpoly.commonrho_(obj.A_, obj.B_, obj.C_, obj.D_, obj.F_);
            % Check dimensions
            sizesNy = [size(obj.A_), size(obj.C_), size(obj.D_), size(obj.F_)];
            if ~all(sizesNy == sizesNy(1))
                error('A, C, D and F should all have the same square size.');
            end
            if obj.Nu > 0 && (size(obj.B_, 1) ~= size(obj.A_, 1))
                error('B must have as many rows as the output size.');
            end
        end
    end
    
    methods (Access = 'protected')
        function sys = extractLocal_(obj, p)
            % EXTRACTLOCAL_ See LPVREP. Returns IDPOLY
            ALocal = freeze(obj.A, p);
            BLocal = freeze(obj.B, p);
            CLocal = freeze(obj.C, p, 'diag');
            DLocal = freeze(obj.D, p, 'diag');
            % If B is empty, F must be empty (idpoly constraint)
            if ~isempty(BLocal)
                FLocal = freeze(obj.F, p, 'F');
            else
                FLocal = [];
            end
            sys = idpoly(ALocal, BLocal, CLocal, DLocal, FLocal, ...
                obj.NoiseVariance, obj.Ts);
        end
    end
    
end

function v = filt2vec_(filt)
    % FILT2VEC_ Convert metadata in cell-array to vector form
    % e.g. filt = {A.Free, B.Free, C.Free, D.Free, F.Free}
    nFilt = numel(filt);
    bigCell = cell(1, nFilt);
    for i=1:nFilt
        mat = filt{i};
        if isa(mat, 'pmatrix')
            mat = mat.matrices;
        end
        [n, ~, ~] = size(mat);
        bigCell{i} = reshape(mat, n, []);
    end
    v = reshape(cell2mat(bigCell), [], 1);
end

function B = makeFiltOrEmpty_(A, mustBeMonic, zeroisnonfree)
    % MAKEFILTOREMPTY_ Convert input to pidpoly or keep empty
    if isa(A, 'pidpoly')
        % If filter should be monic, ensure supplied value is monic
        if mustBeMonic && ~ismonic(A)
            error('Filter must be monic.');
        end
        B = A;
        return;
    end
    if isempty(A); B = []; return; end
    if iscell(A); B = pidpoly(A, [], [], [], mustBeMonic, zeroisnonfree); return; end
    if isa(A, 'pmatrix') || isnumeric(A)
        B = pidpoly({A}, [], [], [], mustBeMonic, zeroisnonfree);
        return;
    end
    error('Could not make pidpoly object.');
end
