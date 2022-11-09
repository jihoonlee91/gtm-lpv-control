classdef idpmatrix
    %IDPMATRIX pmatrix with identifiable coefficients
    
    properties
        Min
        Max
        Free
        Mat
    end
    
    properties (Dependent)
        Value
    end
    
    methods
        function obj = idpmatrix(mat, free, min, max)
            %IDPMATRIX Construct an instance of this class
            if isa(mat, 'idpmatrix')
                obj = mat;
                return;
            end
            obj.Mat = obj2pmatrix(mat);
            [m, n, o] = size(mat);
            if nargin < 2
                free = true(m, n, o);
            end
            if nargin < 3
                min = -inf(m, n, o);
            end
            if nargin < 4
                max = inf(m, n, o);
            end
            obj.Free = free;
            obj.Min = min;
            obj.Max = max;
        end
        
        function val = get.Value(obj)
            val = obj.Mat.matrices;
        end
        
        function D = parder(obj)
            % PARDER Return derivatives with respect to free parameters
            %
            %   Syntax:
            %       D = parder(A)
            %
            %   Inputs:
            %       A: idpmatrix object
            %
            %   Outputs:
            %       D: cell array of idpmatrix objects representing the partial derivative
            %       of A for each free parameter.
            %
            
            % Loop through filters
            D = cell(1, nparams(obj, 'free'));
            DCount = 1;
            free = obj.Free;  % 3-dimensional double
            mat = obj.Mat;    % pmatrix
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
                            D{DCount} = P;
                            DCount = DCount + 1;
                        end
                    end
                end
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
            free = obj.Free;
            mat = obj.Mat;
            [n, m, o] = size(free);
            % Update pmatrix values
            for j=1:o
                for k=1:n
                    for l=1:m
                        if free(k, l, j)
                            mat.matrices(k, l, j) = ...
                                mat.matrices(k, l, j) + delta(1);
                            delta(1) = [];
                        end
                    end
                end
            end
            % Write pmatrix with updated values back
            obj.Mat = mat;
        end
        
        function pvec = getpvec(obj, ~)
            % GETPVEC Values of parameters in vector form
            %
            %   Syntax:
            %       pvec = getpvec(filt)
            %       pvec = getpvec(filt, 'free')
            %
            %   Inputs:
            %       filt: idpmatrix object
            %       'free': whether to only return data for the free
            %           parameters of filt.
            %
            %   Outputs:
            %       pvec: values of the parameters of filt.
            %
            pvec = reshape(obj.Value, [], 1);      % 2D to 1D
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
            if strcmp(mode, 'free')
                val = sum(obj.Free, 'all');
            else
                val = numel(obj.Free);
            end
        end
        
        function f = isconst(obj)
            f = isconst(obj.Mat);
        end
    end
    
    methods (Hidden, Static)
        function [varargout] = commonrho_(varargin)
            %COMMONRHO_ Reformulates input idpmatrix objects to share common ext.
            %scheduling parameter.
            %
            %   Syntax:
            %       [B1, ..., Bn] = commonrho_(A1, ..., An)
            %       [B1, ..., Bn, tm] = commonrho_(A1, ..., An)
            %
            %   Inputs:
            %       A1, ..., An: numeric, pmatrix or idpmatrix objects.
            %
            %   Outputs:
            %       B1, ..., Bn: idpmatrix objects sharing same timemap.
            %       tm: timemap
            %
            
            if nargin ~= nargout && nargin + 1 ~= nargout
                error('Number of output arg. must be number of input args + {1/0}.');
            end

            tm = cell(1, nargin);
            for i=1:nargin
                tm{i} = varargin{i}.Mat.timemap;
            end

            tmCommon = tm{1};
            for i=2:nargin
                tmCommon = union(tmCommon, tm{i});
            end

            for i=1:nargin
                varargin{i}.Mat.timemap = tmCommon;
            end

            if nargout == nargin
                varargout = varargin;
            else
                varargout(1:nargin) = varargin;
                varargout{nargout} = tmCommon;
            end
            
        end
    end
end

