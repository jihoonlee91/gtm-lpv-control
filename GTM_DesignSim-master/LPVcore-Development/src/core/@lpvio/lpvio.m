%LPVIO Linear parameter-varying input-output representation. 
%
%   The LPVIO object will construct the following LPV-IO model:
%
%       A(q,p) y(t) = B(q,p) u(t),
%
%   with polynomials in the backwards-shift operator q^-1 of orders na
%   and nb, as specified in the following equations:
%
%       A(q,p) = sum( A{i} q^-i , i=0,..,na),
%       B(q,p) = sum( B{i} q^-i , i=0,..,nb),
%
%   The A polynomial must be monic.
%
%   Syntax:
%       sys = lpvio(A, B)
%       sys = lpvio(A, B, Ts)
%       sys = lpvio(A, B, Ts, ...)
%
%   Inputs:
%       A: pmatrix object or 1-by-na+1 cell-array of pmatrix objects.
%       B: pmatrix object or 1-by-nb+1 cell-array of pmatrix objects.
%       Ts: sampling time. For continuous-time models, Ts = 0.
%
%   See also PMATRIX
%
classdef (InferiorClasses = {?ss, ?tf, ?zpk}) lpvio < lpvrep
    properties (Dependent)
        A       % 1-by-na+1 cell array of pmatrix objects.
        B       % {}, or 1-by-nb+1 cell array of pmatrix objects
        Na      % numel(A) - 1
        Nb      % numel(B) - 1
    end
    
    properties (Hidden)
        A_
        B_ = {}
    end
    
    methods
        function obj = lpvio(A, B, Ts, varargin)
            %LPVIO Construct an instance of this class
            % Convert inputs to pmatrix format
            %TODO: support empty B matrix
            if iscell(B)
                for i=1:numel(B)
                    obj.B_{i} = obj2pmatrix(B{i});
                end
            else
                obj.B_{1} = obj2pmatrix(B);
            end
            if iscell(A) && numel(A) > 0
                if A{1} ~= eye(size(A{1}))
                    error('A must be monic.');
                end
                for i=1:numel(A)
                    obj.A_{i} = obj2pmatrix(A{i});
                end
            elseif ~isempty(A)
                if A == eye(size(A))
                    obj.A_{1} = obj2pmatrix(A);
                else
                    error('A must be monic.');
                end
            else
                error('lpvio cannot have empty output.');
            end
            % Make ext. scheduling vector common
            [obj.A{:}, obj.B{:}, obj.SchedulingTimeMap] = ...
                pmatrix.commonrho_(obj.A_{:}, obj.B_{:});
            % Set sampling time 
            if nargin == 2
                if strcmp(obj.SchedulingTimeMap.Domain, 'ct')
                    Ts = 0;
                else
                    Ts = -1;
                end
            end
            obj.Ts_ = Ts;
            % Process PV pairs
            p = inputParser;
            p.KeepUnmatched = true;
            parse(p, varargin{:});
            obj = parsePropertyValuePairs(obj, p.Unmatched);
        end
               
        function obj = uplus(obj)
            % UPLUS Unary plus
        end
        
        function [y, t] = lsim(obj, p, u, varargin)
            %LSIM Simulate time reponse of LPV-IO system to arbitrary
            %inputs
            %
            %   Syntax:
            %       [y, t] = lsim(sys, p, u, ...)
            %
            %   Inputs:
            %       sys: CT or DT LPV-IO system
            %       p: N-by-np matrix. Each row corresponds to a time
            %           instant.
            %       u: N-by-nu matrix. Each row corresponds to an input at
            %           that time instant.
            %       t: (CT only) column vector of N equidistant values indicating the
            %           time at which the input u and scheduling variable p is applied.
            %       'p0': M0-by-np matrix, scalar or []. The M0 initial values
            %           of p. M0 is the maximum negative time-shift (DT) or the
            %           maximum derivative order (CT) of the time-map.
            %           Default: 0
            %       'pf': Mf-by-np matrix, scalar or []. The Mf final values of
            %           p. Mf is the maximum positive time-shift (DT) or the
            %           maximum derivative order (CT) of the time-map.
            %           Default: 0
            %       'u0': nb-by-nu matrix, scalar or []. Default: 0
            %       'y0': na-by-ny matrix, scalar or []. Default: 0
            %
            %   Outputs:
            %       y: N-by-ny matrix of simulated output response. 
            %           Each row corresponds to a time instant.
            
            lsimOpts = LPVcore.parselsimOpts(obj, p, u, varargin{:});
            
            if isct(obj)
                [y, t] = ctsim_(obj, p, u, lsimOpts.t, lsimOpts.Solver, ...
                    lsimOpts.SolverOpts, 'fast');
                [y2, ~] = ctsim_(obj, p, u, lsimOpts.t, lsimOpts.Solver, ...
                    lsimOpts.SolverOpts, 'accurate');
                assert(all(y-y2 < eps(y), 'all'), 'fast doesnt match accurate')
            else
                if strcmp(lsimOpts.solverMode, 'fast')
                    [y, t] = dtsim_MatWise(obj, p, u, lsimOpts.p0, lsimOpts.pf, ...
                            lsimOpts.u0, lsimOpts.y0);
                elseif strcmp(lsimOpts.solverMode, 'large')
                    [y, t] = dtsim_(obj, p, u, lsimOpts.p0, lsimOpts.pf, ...
                            lsimOpts.u0, lsimOpts.y0);
                end
            end
            if nargout == 0
                figure;
                plotlsim(obj, y, p, u, t);
            end
        end
        
        function val = getNy(obj)
            if numel(obj.A_) > 0
                val = size(obj.A_{1}, 1);
            else
                val = size(obj.B_{1}, 1);
            end
        end
        
        function val = getNu(obj)
            if numel(obj.B_) > 0
                val = size(obj.B_{1}, 2);
            else
                val = 0;
            end
        end
        
        function val = get.Na(obj)
            val = numel(obj.A_) - 1;
        end
        
        function val = get.Nb(obj)
            val = numel(obj.B_) - 1;
        end
        
        function A = get.A(obj); A = obj.A_; end
        function B = get.B(obj); B = obj.B_; end
        function obj = set.A(obj, A)
            obj = updatePMat_(obj, 'A_', A, obj.Ny, obj.Ny);
            if obj.A_{1} ~= eye(size(obj.A_{1}))
                error('A must be monic.');
            end
        end
        
        function obj = set.B(obj, B)
            obj = updatePMat_(obj, 'B_', B, obj.Ny, obj.Nu);
        end
    end
    
    methods (Hidden, Access = 'protected')
        function sys = extractLocal_(obj, p)
            % EXTRACTLOCAL_ See LPVREP. Returns TF.
            ALocal = cell(1, obj.Na + 1);
            BLocal = cell(1, obj.Nb + 1);
            for i=1:obj.Na+1
                ALocal{i} = freeze(obj.A{i}, p);
            end
            for i=1:obj.Nb+1
                BLocal{i} = freeze(obj.B{i}, p);
            end
            if strcmp(obj.SchedulingTimeMap.Domain, 'ct')
                xi = tf('s');
            else
                xi = 1 / tf('z', obj.Ts);
            end
            ALTI = zeros(obj.Ny);
            for i=1:obj.Na+1
                ALTI = ALTI + ALocal{i} * xi^(i-1);
            end
            BLTI = zeros(obj.Ny, obj.Nu);
            for i=1:obj.Nb+1
                BLTI = BLTI + BLocal{i} * xi^(i-1);
            end
            sys = tf(ALTI \ BLTI);
        end
        
        function obj = updatePMat_(obj, name, val, nVal, mVal)
            % UPDATEPMAT Update pmatrix of this representation after
            % validating size
            %
            %   Syntax:
            %       obj = updatePMat(obj, name, val, n, m)
            %
            %   Inputs:
            %       name: name of property to update ('A_', 'B_')
            %       val: pmatrix object or cell array of pmatrix objects to set
            %       n, m: valid size
            %
            if ~iscell(val)
                val = {val};
            end
            % Validate size
            assert(LPVcore.validatepmatrixCellArray(val), ...
                ['Attempting to create LPVIO object ', ...
                'with PMATRIX objects of different size.']);
            for i=1:numel(val)
                [n, m] = size(val{i});
                if ~all([n, m] == [nVal, mVal])
                    error([name(1), ' matrix must be ', ...
                        int2str(nVal), '-by-', int2str(mVal)]);
                end
            end
            obj.(name) = val;
            obj = makeCommonRho_(obj);
        end
        
        function obj = makeCommonRho_(obj)
            [obj.A_{:}, obj.B_{:}, obj.SchedulingTimeMap] = ...
                pmatrix.commonrho_(obj.A_{:}, obj.B_{:});
        end
    end
end

