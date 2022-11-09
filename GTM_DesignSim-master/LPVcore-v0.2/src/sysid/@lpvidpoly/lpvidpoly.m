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
        A               % Monic lpvidfilt
        B               % lpvidfilt
        C               % Monic lpvidfilt
        D               % Monic lpvidfilt
        F               % Monic lpvidfilt
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
            isValidFilter = @(x) isa(x, 'lpvidfilt') ...
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
            
            % Cast input to lpvidfilt object
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
            if isempty(A); A = lpvidfilt({eye(ny)}, [], [], [], true); end
            if isempty(B); B = lpvidfilt({[]}, [], [], [], false); end
            if isempty(C); C = lpvidfilt({eye(ny)}, [], [], [], true); end
            if isempty(D); D = lpvidfilt({eye(ny)}, [], [], [], true); end
            if isempty(F); F = lpvidfilt({eye(ny)}, [], [], [], true); end

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
        
        function obj = applydeltap(obj, delta)
            % APPLYDELTAP Update values of free parameters in vector form
            %
            %   Syntax:
            %       obj = getpvec(obj, delta)
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
            if ~isa(A, 'lpvidfilt'); A = lpvidfilt(A, [], [], [], true, obj.ZeroIsNonFree); end
            if ~ismonic(A); error('A must be monic'); end
            obj.A_ = A;
            obj = updateFilt_(obj);
        end
        
        function obj = set.B(obj, B)
            if ~isa(B, 'lpvidfilt'); B = lpvidfilt(B, [], [], [], false, obj.ZeroIsNonFree); end
            obj.B_ = B;
            obj = updateFilt_(obj);
        end
        
        function obj = set.C(obj, C)
            if ~isa(C, 'lpvidfilt'); C = lpvidfilt(C, [], [], [], true, obj.ZeroIsNonFree); end
            if ~ismonic(C); error('C must be monic'); end
            obj.C_ = C;
            obj = updateFilt_(obj);
        end
        
        function obj = set.D(obj, D)
            if ~isa(D, 'lpvidfilt'); D = lpvidfilt(D, [], [], [], true, obj.ZeroIsNonFree); end
            if ~ismonic(D); error('D must be monic'); end
            obj.D_ = D;
            obj = updateFilt_(obj);
        end
        
        function obj = set.F(obj, F)
            if ~isa(F, 'lpvidfilt'); F = lpvidfilt(F, [], [], [], true, obj.ZeroIsNonFree); end
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
        
        function tf = isarx(obj)
            % ISARX Returns logical 1 (true) if object represents LPV-ARX
            % system (C, D, F == eye)
            tf = iseye(obj.C) && iseye(obj.D) && iseye(obj.F);
        end
        
        function tf = isarmax(obj)
            % ISARMAX Returns logical 1 (true) if object represents
            % LPV-ARMAX system (D, F == eye)
            tf = iseye(obj.D) && iseye(obj.F);
        end
        
        function tf = isoe(obj)
            % ISOE Returns logical 1 (true) if object represents LPV-OE
            % system (A, C, D == eye)
            tf = iseye(obj.A) && iseye(obj.C) && iseye(obj.D);
        end
        
        function tf = isbj(obj)
            % ISBJ Returns logical 1 (true) if object represents LPV-BJ
            % system (A == eye)
            tf = iseye(obj.A);
        end
        
        function name = modelStructureName(obj)
            % MODELSTRUCTURENAME Return name of model structure ('ARX',
            % 'ARMAX', 'OE' or 'BJ'
            %
            %   Syntax:
            %       name = modelStructureName(obj)
            %
            if isarx(obj); name = 'ARX'; return; end
            if isarmax(obj); name = 'ARMAX'; return; end
            if isoe(obj); name = 'OE'; return; end
            if isbj(obj); name = 'BJ'; return; end
            name = 'IDPOLY';
        end
        
        function val = get.Type(obj); val = modelStructureName(obj); end
    end
    
    methods (Access = 'private')
    
        function obj = updateFilt_(obj)
            % Get common ext. scheduling variable and timemap
            [obj.A_, obj.B_, obj.C_, obj.D_, obj.F_, obj.SchedulingTimeMap] = ...
                lpvidfilt.commonrho_(obj.A_, obj.B_, obj.C_, obj.D_, obj.F_);
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
            CLocal = freeze(obj.C, p);
            DLocal = freeze(obj.D, p);
            FLocal = freeze(obj.F, p);
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
    % MAKEFILTOREMPTY_ Convert input to lpvidfilt or keep empty
    if isa(A, 'lpvidfilt')
        % If filter should be monic, ensure supplied value is monic
        if mustBeMonic && ~ismonic(A)
            error('Filter must be monic.');
        end
        B = A;
        return;
    end
    if isempty(A); B = []; return; end
    if iscell(A); B = lpvidfilt(A, [], [], [], mustBeMonic, zeroisnonfree); return; end
    if isa(A, 'pmatrix') || isnumeric(A)
        B = lpvidfilt({A}, [], [], [], mustBeMonic, zeroisnonfree);
        return;
    end
    error('Could not make lpvidfilt object.');
end
