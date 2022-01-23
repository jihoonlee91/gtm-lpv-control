classdef (Abstract) pbasis
    %PBASIS Abstract class for parameter-varying basis functions
    %
    % Parametery-varying basis functions map an extended scheduling vector
    % (rho1, ..., rhoN) to a scalar. They are used to construct PMATRIX
    % objects.
    %
    % Do not create PBASIS objects directly. Instead, use PMATRIX or PREAL
    % to define parameter-varying matrices or scalars.
    %
    properties
       sortKey
       dependenceType
    end
    
    methods
        function c = lt(A, B)
            c = A.sortKey < B.sortKey;
        end
        
        function c = gt(A, B)
            c = lt(B, A);
        end

        function C = mtimes(A, B)
            C = times(A, B);
        end
        
        function A = diff(A, order, schedulingDimension)
            % DIFF Apply higher-order derivative
            %
            %   Syntax:
            %       A = diff(A, order, schedulingDimension)
            %
            %   Inputs:
            %       order: order of the derivative (must be non-negative
            %       integer)
            %
            %       schedulingDimension: dimension of the scheduling
            %       variable (required for re-indexing the basis function
            %       in terms of the ext. scheduling variable)
            %
            if round(order) ~= order || order < 0
                error('Order must be non-negative integer');
            end
            % Higher-order derivative is executed by repeatedly performing
            % first-order derivative
            for i=1:order
                A = diff_(A, schedulingDimension);
            end
        end
        
        function c = eq(A, B)
            c = strcmp(A.sortKey, B.sortKey);
        end
        
        function c = ne(A, B)
            c = ~eq(A, B);
        end
        
        s = str(obj, rhoNames)
        
        
        function minNrho = minNrho(obj, varargin)
            %Returns the minimum required number of input rhos to evaluate the
            %basis function(s).
            %
            %   Syntax:
            %       N = minNRho(bfunc)
            %       N = minNRho(pmatrix.bfuncs{:})
            %
            %   Input
            %       bfunc: a single basisfunction.
            %       bfuncs: a cell array of basisfunctions
            %
            minNrho = minNRho(obj, varargin{:});
        end
            
        function disp(obj, rhoNames)
            fprintf('Basis function with %s scheduling dependence:\n\n', ...
                obj.dependenceType);
            if nargin == 1
                fprintf(['\tphi = ', str(obj), '\n\n']);
            else
                fprintf(['\tphi = ', str(obj, rhoNames), '\n\n']);
            end
        end

    end
    
    methods (Hidden)
        function key = sortkey(obj)
            warning('sortkey() is a part of the legacy code, please use obj.sortKey')
            try
                key = obj.sortKey;
            catch
                key = -1;
            end
        end

    end
    
    methods (Hidden, Abstract, Access=protected)
        A = diff_(A, schedulingDimension);
    end
end

