classdef idpmatrix < pmatrix
    %IDPMATRIX pmatrix with identifiable coefficients
    %
    % The idpmatrix object extends the functionality of pmatrix with
    % additional attributes on the coefficients. These attributes are
    % useful for identification purposes:
    %   * Free: logical denoting which coefficients are free.
    %   * Max/Min: the max. and min. values of the coefficients.
    %
    % Syntax:
    %
    %   p = idpmatrix(mat, free, min, max, bfuncs, Name, Value)
    %   p = idpmatrix(mat, [], [], [], bfuncs, Name, Value)
    %   
    % creates a new idpmatrix object. The input arguments mat, bfuncs and the Name, Value
    % pairs are identical to the standard pmatrix object. The input
    % arguments free, min and max are arrays of the same size as mat and
    % determine the corresponding properties of each coefficients of the
    % scheduling dependent matrix. Setting them empty ([]) fills in the
    % default values.
    %
    % Syntax:
    %
    %   p = idpmatrix(p0)
    %
    % converts a standard pmatrix object p0 into an idpmatrix object. By
    % default, all coefficients are set to free, with min. value -Inf and
    % max. value +Inf. To set different defaults, use the following syntax:
    %
    %   p = idpmatrix(p0, free, min, max)
    %
    % See also PMATRIX
    %
    
    properties
        Min
        Max
        Free
    end

    properties (Dependent)
        Value
    end
    
    methods
        function obj = idpmatrix(mat, free, min, max, varargin)
            %IDPMATRIX Construct an instance of this class
            
            % Handle conversion from pmatrix to idpmatrix
            if strcmp(class(mat), 'pmatrix') %#ok<STISA> 
                pmat = mat;
                mat = pmat.matrices;
                varargin = {pmat.bfuncs, 'SchedulingTimeMap', pmat.timemap};
            end

            obj@pmatrix(mat, varargin{:});
            if nargin <= 1 || isempty(free)
                if ~isa(mat, 'idpmatrix')
                    free = true(size(obj.matrices));
                else
                    free = mat.Free;
                end
            end
            if nargin <= 2 || isempty(min)
                if ~isa(mat, 'idpmatrix')
                    min = -Inf(size(obj.matrices));
                else
                    min = mat.Min;
                end
            end
            if nargin <= 3 || isempty(max)
                if ~isa(mat, 'idpmatrix')
                    max = +Inf(size(obj.matrices));
                else
                    max = obj.Max;
                end
            end
            obj.Free = free;
            obj.Min = min;
            obj.Max = max;
        end
        
        function disp(obj)
            % DISP Print object information to screen
            if isempty(obj)
                disp('Empty scheduling dependent matrix with identifiable coefficients');
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
                disp( [ domain , 'scheduling dependent matrix with identifiable coefficients'] )
                disp( ' ' )
                fprintf(['\t', symbolRepr])
                fprintf('\n\n');
            else
                disp( [ domain , 'scheduling dependent ' ,num2str(sz1), 'x', ...
                    num2str(sz2),' matrix with identifiable coefficients'] )
                disp( ' ' )
                fprintf(['\t', symbolRepr])
                fprintf('\n\n')
                dispcoeff(obj, 'A');
            end

            % Display information on number of parameters
            fprintf('Parameterization:\n');
            fprintf('\tNumber of coefficients: %d (free: %d)\n', ...
                nparams(obj), nparams(obj, 'free'));
        end

        function val = get.Value(obj)
            val = obj.matrices;
        end

        function obj = set.Free(obj, value)
            if isscalar(value)
                value = value * true(size(obj.matrices));
            end
            obj.Free = value;
        end

        function obj = set.Min(obj, value)
            if isscalar(value)
                value = value * ones(size(obj.matrices));
            end
            obj.Min = value;
        end

        function obj = set.Max(obj, value)
            if isscalar(value)
                value = value * ones(size(obj.matrices));
            end
            obj.Max = value;
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
            mat = obj;    % pmatrix
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
            mat = obj;
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
            obj = mat;
        end

        function [obj, p] = setpvec(obj, p)
            % SETPVEC Set free parameters
            %
            %   Syntax:
            %       [obj, p] = setpvec(obj, p)
            %
            %   Inputs:
            %       p: N-by-1 vector of free parameter
            %       updates (corresponding e.g. to a single gradient step).
            %       If N > nparams(obj, 'free'), then only the first
            %       nparams(obj, 'free') elements will be used.
            %
            %   Outputs:
            %       p: (N-nparams(obj, 'free'))-by-1 vector of free
            %       parameters that were not used to update this particular
            %       filter.
            
            % Input validation
            if numel(p) < nparams(obj, 'free')
                error('Too few parameters were provided to update values.');
            end
            free = obj.Free;
            mat = obj;
            [n, m, o] = size(free);
            % Update pmatrix values
            for j=1:o
                for k=1:n
                    for l=1:m
                        if free(k, l, j)
                            mat.matrices(k, l, j) = p(1);
                            p(1) = [];
                        end
                    end
                end
            end
            % Write pmatrix with updated values back
            obj = mat;
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
            pvec = obj.Value(:);      % 2D to 1D
            if nargin > 1
                pvec = pvec(obj.Free(:) == true);
            end
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
    end
end

