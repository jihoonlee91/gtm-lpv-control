classdef lpvidss < lpvrep & lpvidsys
    %LPVIDSS State-space LPV model with identifiable parameters
    %
    %   The LPVIDSS class represents a linear parameter-varying state-space
    %   representation in the innovation form:
    %
    %       dx(t)/dt = A(rho(t)) x(t) + B(rho(t)) u(t) + K(rho(t)) e(t),
    %         y(t)   = C(rho(t)) x(t) + D(rho(t)) u(t) + e(t),
    %
    %    or the state-space form with general noise model:
    %
    %       dx(t)/dt = A(rho(t)) x(t) + B(rho(t)) u(t) + G(rho(t)) v(t),
    %         y(t)   = C(rho(t)) x(t) + D(rho(t)) u(t) + H(rho(t)) e(t),
    %
    %   where the white noise processes are normally distributed, i.e.,
    %
    %    e(t) ~ N(0,NoiseVariance)   or    [v(t);e(t)]  ~ N(0,NoiseVariance).
    %
    %
    %   Syntax:
    %    sys = lpvidss(A,B,C)
    %    sys = lpvidss(A,B,C,D)
    %    sys = lpvidss(A,B,C,D,'general',G,H)
    %    sys = lpvidss(A,B,C,D,'general',G,H,NoiseVariance)
    %    sys = lpvidss(A,B,C,D,'general',G,H,NoiseVariance,Ts)
    %    sys = lpvidss(A,B,C,D,'general',G,H,NoiseVariance,Ts,'Property1',Value1,...,'PropertyN',ValueN)
    %    sys = lpvidss(A,B,C,D,'innovation',K)
    %    sys = lpvidss(A,B,C,D,'innovation',K, [], NoiseVariance)
    %    sys = lpvidss(A,B,C,D,'innovation',K, [], NoiseVariance,Ts)
    %    sys = lpvidss(A,B,C,D,'innovation',K, [], NoiseVariance,Ts,'Property1',Value1,...,'PropertyN',ValueN)
    %
    %
    %   A is a matrix/pmatrix. A needs to be square with dimension nx-by-nx
    %     where nx is the state dimension. Default: []. 
    %   B is a matrix/pmatrix. B is of dimension nx-by-nu where nu is the input
    %     dimension and nx is the state dimension. Default: [].
    %   C is a matrix/pmatrix. C is of dimension ny-by-nx where ny is the
    %     output dimension and nx is the state dimension. Default: [].
    %   D is a matrix/pmatrix. D is of dimension ny-by-nu where nu is the input
    %     dimension and ny is the output dimension. Default: [].
    %   K is a matrix/pmatrix. K is of dimension nx-by-ny where ny is the
    %     output dimension and nx is the state dimension. Default: [].
    %   G is a matrix/pmatrix. G needs to be square with dimension nx-by-nx
    %     where nx is the state dimension. Default: []. 
    %   H is a matrix/pmatrix. H needs to be square with dimension ny-by-ny
    %     where ny is the output dimension. Default: []. 
    %   NoiseVariance is a positive definite matrix. For the innovation model,
    %     the NoiseVariance is of dimension ny-by-ny where ny is the output
    %     dimension. For the general noise model, the NoiseVariance is of
    %     dimension (nx+ny)-by-(nx+ny) where ny is the output dimension and nx
    %     is the state dimension. Default: 1.
    %   Ts is the sampling time. For continuous-time models, Ts = 0. For
    %     discrete-time models, Ts is a positive scalar representing the
    %     sampling period (unknown sampling time Ts = -1).
    %
    %   See also LPVREP, LPVSS, LPVIO, LPVIDPOLY, PMATRIX, TIMEMAP.
    
    properties
        % Scalar or Ny-by-Ny matrix. Default: eye(Ny)
        NoiseVariance
        % Parameter covariance matrix
        Covariance
        % Mode type ('SS')
        Type
    end
    
    properties (Dependent)
        A               % pmatrix
        B               % pmatrix
        C               % pmatrix
        D               % pmatrix
        K               % innovation form only, pmatrix
        G               % pmatrix
        H               % pmatrix
        Structure       % Information on model structure
        Nx              % int, number of states
        X0              % initial state
    end
    
    properties (Access = private)
        A_ = []
        B_ = []
        C_ = []
        D_ = []
        K_ = []
        G_ = []
        H_ = []
        X0_ = []
        IsInnovationForm_   % logical indicating general (false) or innovation form (true)
    end
    
    methods
        function obj = lpvidss(varargin)
            %LPVIDSS Construct an instance of this class
            %
            %   See class description.
            %
            isValidMatrix = @(x) isa(x, 'pmatrix') || isnumeric(x);
            
            p = inputParser();
            p.KeepUnmatched = true;
            addOptional(p, 'A', [], @(x) isValidMatrix(x));
            addOptional(p, 'B', [], @(x) isValidMatrix(x));
            addOptional(p, 'C', [], @(x) isValidMatrix(x));
            addOptional(p, 'D', [], @(x) isValidMatrix(x));
            addOptional(p, 'form', 'general', ...
                @(x) any(strcmpi(x, {'general', 'innovation'})));
            addOptional(p, 'G', [], @(x) isValidMatrix(x));  % K for innovation form
            addOptional(p, 'H', [], @(x) isValidMatrix(x));
            addOptional(p, 'NoiseVariance', 1, @(x) isnumeric(x) || isempty(x));
            addOptional(p, 'Ts', -1, @(x) isnumeric(x) && (isscalar(x) || isempty(x)));
            parse(p, varargin{:});
            
            A = p.Results.A;
            B = p.Results.B;
            C = p.Results.C;
            D = p.Results.D;
            G = p.Results.G;  % K for innovation form
            K = G;
            H = p.Results.H;
            obj.IsInnovationForm_ = strcmp(p.Results.form, 'innovation');
            if obj.IsInnovationForm_
                nx = max([size(A, 1), size(K, 1)]);
                nu = max([size(B, 2), size(D, 2)]);
                ny = max([size(C, 1), size(D, 1), size(K, 2)]);
                H = eye(ny);
            else
                nx = max([size(A, 1), size(G, 1)]);
                nu = max([size(B, 2), size(D, 2)]);
                ny = max([size(C, 1), size(D, 1), size(H, 1)]);
            end
            % Populate empty entries
            if isempty(A); A = zeros(nx); end
            if isempty(B); B = zeros(nx, nu); end
            if isempty(C); C = zeros(ny, nx); end
            if isempty(D); D = zeros(ny, nu); end
            if obj.IsInnovationForm_
                if isempty(G); G = zeros(nx, ny); end
            else
                if isempty(G); G = zeros(nx); end
                if isempty(H); H = zeros(ny); end
            end
            
            % Convert to idpmatrix
            A = idpmatrix(A);
            B = idpmatrix(B);
            C = idpmatrix(C);
            D = idpmatrix(D);
            G = idpmatrix(G);
            H = idpmatrix(H);
            
            [obj.A_, obj.B_, obj.C_, obj.D_, obj.G_, obj.H_, obj.SchedulingTimeMap] = ...
                idpmatrix.commonrho_(A, B, C, D, G, H);
            
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
            if isempty(Ts); Ts = -1; end
            if strcmp(obj.SchedulingTimeMap.Domain, 'ct') && Ts > 0
                error('Ts must be 0 for CT model.');
            elseif strcmp(obj.SchedulingTimeMap.Domain, 'dt') && Ts == 0
                error('Ts cannot be 0 for DT model.');
            end
            obj.Ts = Ts;
            
            % Initial state defaults to 0
            obj.X0 = zeros(obj.Nx, 1);
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
            val = val + nparams(obj.G_, f);
            val = val + nparams(obj.H_, f);
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
            %       sys: lpvidss object
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
            pvecA = getpvec(obj.A_, f);
            pvecB = getpvec(obj.B_, f);
            pvecC = getpvec(obj.C_, f);
            pvecD = getpvec(obj.D_, f);
            pvecG = getpvec(obj.G_, f);
            pvecH = getpvec(obj.H_, f);
            pvec = [pvecA; pvecB; pvecC; pvecD; pvecG; pvecH];
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
            [obj.K_, delta] = applydeltap(obj.G_, delta);
            obj.X0 = obj.X0 + delta;
        end
        
        function val = get.A(obj); val = obj.A_.Mat; end
        function val = get.B(obj); val = obj.B_.Mat; end
        function val = get.C(obj); val = obj.C_.Mat; end
        function val = get.D(obj); val = obj.D_.Mat; end
        function val = get.G(obj)
            assert(~obj.IsInnovationForm_, ...
                'G is not available for innovation form LPVIDSS models.');
            val = obj.G_.Mat;
        end
        function val = get.H(obj)
            assert(~obj.IsInnovationForm_, ...
                'H is not available for innovation form LPVIDSS models.');
            val = obj.H_.Mat;
        end
        function val = get.X0(obj); val = obj.X0_; end
        
        function val = get.K(obj)
            if ~obj.IsInnovationForm_
                warning('Use G for general form LPVIDSS models.');
            end
            val = obj.G_.Mat;
        end
        
        function obj = set.A(obj, A)
            obj.A_ = idpmatrix(A);
        end
        
        function obj = set.B(obj, B)
            obj.B_ = idpmatrix(B);
        end
        
        function obj = set.C(obj, C)
            obj.C_ = idpmatrix(C);
        end
        
        function obj = set.D(obj, D)
            obj.D_ = idpmatrix(D);
        end
        
        function obj = set.K(obj, K)
            if ~obj.IsInnovationForm_
                warning('Use G for general form LPVIDSS models.');
            end
            obj.G_ = idpmatrix(K);
        end
        
        function obj = set.G(obj, G)
            assert(~obj.IsInnovationForm_, ...
                'G is not available for innovation form LPVIDSS models.');
            obj.G_ = idpmatrix(G);
        end
        
        function obj = set.H(obj, H)
            assert(~obj.IsInnovationForm_, ...
                'H is not available for innovation form LPVIDSS models.');
            obj.H_ = idpmatrix(H);
        end
        
        function obj = set.X0(obj, val)
            assert(isnumeric(val) && numel(val) == obj.Nx, ...
                'X0 must be vector with same number of elements as state dimension.');
            assert(size(val, 1) == 1 || size(val, 2) == 1, ...
                'X0 must be a vector.');
            if size(val, 2) > size(val, 1)
                val = val';
            end
            obj.X0_ = val;
        end
        
        function val = getNu(obj); val = size(obj.B_.Mat, 2); end
        function val = getNy(obj); val = size(obj.C_.Mat, 1); end
        function val = get.Nx(obj); val = size(obj.A_.Mat, 1); end
        
        function val = get.Type(~); val = 'SS'; end
        
        function val = get.Structure(obj)
            val.A = obj.A_;
            val.B = obj.B_;
            val.C = obj.C_;
            val.D = obj.D_;
            if obj.IsInnovationForm_
                val.K = obj.G_;
            else
                val.G = obj.G_;
                val.H = obj.H_;
            end
            % TODO: add StateName and StateUnit
        end
        
        function val = order(obj)
            % ORDER Return dimension of the state
            val = obj.Nx;
        end
    end
    
    methods (Hidden)
        function sys_equiv = equivlpvss(obj)
            a = obj.A_.Mat;
            b = obj.B_.Mat;
            c = obj.C_.Mat;
            d = obj.D_.Mat;
            g = obj.G_.Mat;
            h = obj.H_.Mat;
            sys_equiv = lpvss(a, [b, g, zeros(obj.Nx, obj.Nu)], ...
                c, [d, zeros(obj.Ny, obj.Nu), h], obj.Ts);
        end
    end
    
    methods (Access = 'private')
    
        function obj = updateFilt_(obj)
            % Get common ext. scheduling parameter and timemap
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
    
    methods (Hidden, Access = 'protected')
        function sys = extractLocal_(obj, p)
            % EXTRACTLOCAL_ See LPVREP. Returns IDSS
            assert(obj.IsInnovationForm_, ...
                'Local model extraction is only supported for innovation form.');
            ALocal = freeze(obj.A, p);
            BLocal = freeze(obj.B, p);
            CLocal = freeze(obj.C, p);
            DLocal = freeze(obj.D, p);
            KLocal = freeze(obj.K, p);
            sys = idss(ALocal, BLocal, CLocal, DLocal, KLocal, ...
                'NoiseVariance', obj.NoiseVariance, ...
                'Ts', obj.Ts);
        end
    end
    
end