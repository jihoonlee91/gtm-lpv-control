classdef gridvar
    %GRIDVAR Grid-based variable
    %
    % Represents a grid-based matrix specified at a number of samples.
    % A GRIDVAR object is created using the following syntax:
    %
    %   P = gridvar(mat)
    %
    % where mat is a n x m x p matrix, which creates a n x m GRIDVAR object
    % specified at p samples. If value at each grid point is the same, the
    % gridvar is automatically simplified to have only one grid point.
    %
    % Example:
    % 
    %   >> P = gridvar(cat(3,eye(2),eye(2)*2,eye(2)*3))
    % 
    %   P = 
    %
    %   Grid-based 2x2 matrix with 3 samples
    %
    %   [Sample 1]
    %        1     0
    %        0     1
    %   
    %   [Sample 2]
    %        2     0
    %        0     2
    %   
    %   [Sample 3]
    %        3     0
    %        0     3    
    %
    % The input matrix can also be a sdpvar or ndsdpvar object.
    % Example:
    %
    %     >> P = gridvar(sdpvar(2,2,5,'sym'))
    %     
    %     P = 
    %     
    %     Grid-based 2x2 matrix with 5 samples
    %     
    %     [Sample 1]
    %            sdpvar object: 2-by-2
    %     
    %     [Sample 2]
    %            sdpvar object: 2-by-2
    %     
    %     [Sample 3]
    %            sdpvar object: 2-by-2
    %     
    %     [Sample 4]
    %            sdpvar object: 2-by-2
    %     
    %     [Sample 5]
    %            sdpvar object: 2-by-2
    %
    properties (SetAccess='private')
        matrices    % matrices
        Ng          % total number of grid points
    end

    properties (Hidden, SetAccess='private')
        dim         % dimensions of the grid
    end
    
    methods
        function obj = gridvar(A)
            %GRIDVAR Construct an instance of this class
            %
            %   Syntax:
            %       P = gridvar(mat)
            %
            %   Inputs:
            %       mat (double,sdpvar,ndsdpvar): Matrix or 3D tensor
            %           specifying the matrices.
            %   
            %   Outputs:
            %       P: gridvar object.
            %
            if ~isa(A,'gridvar')
                assert(isa(A,'double') || ...
                       isa(A,'sdpvar') || ...
                       isa(A,'ndsdpvar'), ...
                       'Unsupported input type.')
                dim = size(A);
                
                obj.dim = dim(3:end);
                if isempty(obj.dim) 
                    obj.dim = 1;
                end

                obj.Ng = prod(obj.dim);
                
                dim = [dim(1:2),obj.Ng];
                dimCell = {dim(1),dim(2),ones(obj.Ng,1)};
                obj.matrices = squeeze(mat2cell(reshape(A,dim),dimCell{:}));

                obj = simplify(obj);
            else
                obj = A;
            end
        end

        function A = plus(A, B)
            A = twoArgFun(A, B, @plus);
        end
        
        function A = uminus(A)
            A = oneArgFun(A, @uminus);
        end
        
        function C = minus(A, B)
            C = plus(A, -B);
        end
        
        function A = mtimes(A,B)
            A = twoArgFun(A, B, @mtimes);
        end

        function A = times(A,B)
            A = twoArgFun(A, B, @times);
        end

        function A = ldivide(A,B)
            A = twoArgFun(A, B, @ldivide);
        end

        function A = rdivide(A,B)
            A = twoArgFun(A, B, @rdivide);
        end

        function A = mldivide(A,B)
            A = twoArgFun(A, B, @mldivide);
        end

        function A = mrdivide(A,B)
            A = twoArgFun(A, B, @mrdivide);
        end

        function A = ctranspose(A)
            A = oneArgFun(A, @ctranspose);
        end
        
        function A = transpose(A)
            A = oneArgFun(A, @transpose);
        end

        function A = inv(A)
            A = oneArgFun(A, @inv);
        end
        
        function C = vertcat(varargin)
            % VERTCAT Vertical concatenation
            C = cat(1, varargin{:});
        end
        
        function C = horzcat(varargin)
            % HORZCAT Horizontal concatenation
            C = cat(2, varargin{:});
        end

        function C = cat(dim, varargin)
            % CAT Concatenation
            [varargin{1:nargin-1}] = commongrid_(varargin{:});
            C = varargin{1};
            
            % iteratively concatenate pairs of gridvar objects
            for i=2:nargin-1
                C = cat_(dim, C, varargin{i});
            end
            C = simplify(C);
        end
        
        function C = blkdiag(varargin)
            % BLKDIAG Block diagonal concatenation
            [varargin{:}] = commongrid_(varargin{:});
            C = varargin{1};
            
            % iteratively blkdiag pairs of gridvar objects
            for i=2:nargin
                C = blkdiag_(C, varargin{i});
            end
            C = simplify(C);
        end
        
        function v = diag(v)
            v = oneArgFun(v, @diag);
        end

        function A = kron(A,B)
            A = twoArgFun(A, B, @kron);
        end

        function N = null(A)
            N = oneArgFun(A, @null);
        end

        function LMI = le(A, B)
            %LE 
            [A,B] = commongrid_(A,B);
            assert(isa(A.matrices{1},'sdpvar') || ...
                isa(B.matrices{1},'sdpvar'), ['One of the gridvar '...
                'should have sdpvars as gridpoints']);
            
            LMI = [];
            for i = 1:A.Ng
                LMI = LMI + (A.matrices{i} <= B.matrices{i});
            end
        end

        function LMI = ge(A, B)
            LMI = le(-A, -B);
        end

        function LMI = lt(A, B)
            warning(['Strict inequalities are not supported. ',...
                'A non-strict has been added instead.'])
            LMI = le(A,B);
        end

        function LMI = gt(A, B)
            LMI = lt(-A, -B);
        end

        function varargout = size(obj, varargin)
            % SIZE Return size of gridvar
            %
            %   Syntax:
            %       sz = size(P)
            %       szdim = size(P, dim)
            %       [sz1, sz2, sz3] = size(__)
            %
            %   Inputs:
            %       P: gridvar objects
            %       dim: (int) dimension along which to return the size.
            %       Valid values are 1 and 2 for the sizes of the
            %       matrix-valued coefficients, and 3 for the number of
            %       samples/grid-points.
            %
            %   Outputs:
            %       sz: row vector with size of P along first two
            %       dimensions.
            %       szdim: size along the requested dimension.
            %       sz1, ..., sz3: size along each of the three dimensions
            %       returned as separate integers.
            %
            sz = [size(obj.matrices{1}, 1), size(obj.matrices{1}, 2), obj.Ng];
            if nargin == 2
                dims = varargin{1};
                if dims > 3
                    varargout{1} = 1;
                else
                    varargout{1} = sz(dims);
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

        function n = numel(obj)
            % NUMEL Return number of elements of gridvar
            %
            %   Syntax:
            %       n = numel(P)
            %
            %   Inputs:
            %       P: gridvar objects
            %
            %   Outputs:
            %       n: Number of elements of P at a sample/grid-point.
            %
            n = numel(obj.matrices{1});
        end

        function disp(obj)
            % DISP Print object information to screen
            [n,m] = size(obj);
            fprintf('Grid-based %ix%i matrix with %i samples',n,m,obj.Ng);
            if numel(obj.dim) > 1
                fprintf([' on ',repmat('%ix',1,numel(obj.dim)-1),'%i grid'],obj.dim)
            end
            fprintf('\n\n');
            if obj.Ng > 10
                for i = 1:5
                    fprintf('[Sample %i]\n',i);
                    disp(obj.matrices{i});
                end
                fprintf('\t.\n\t.\n\t.\n\n');
                for i = (obj.Ng - 5):obj.Ng
                    fprintf('[Sample %i]\n',i);
                    disp(obj.matrices{i});
                end
            else
                for i = 1:obj.Ng
                    fprintf('[Sample %i]\n',i);
                    disp(obj.matrices{i});
                end
            end
        end

        function B = double(A)
            % Overload from sdpvar/double
            B = value(A);
        end

        function B = value(A)
            % Overload from sdpvar/value
            if A.Ng > 1
                B = A;
                mat = cellfun(@(A) value(A),A.matrices,'UniformOutput',false);
                B.matrices = mat;
            else
                B = value(A.matrices{1});
            end
        end

        function A = simplify(A)
            %SIMPLIFY simplifies gridvar object to one gridpoint if value
            %at each gridpoint is the same.
            %   
            %   Syntax:
            %       B = simpify(A);
            % 

            if A.Ng == 1 || isa(A.matrices{1},'sdpvar')
                return;
            else
                mat = getMatrices(A);
                if all(mat == mat(:,:,1), 'all')
                    A = gridvar(mat(:,:,1));
                end
            end
        end
    end

    methods (Hidden)
        % Required for subsref:
        % See https://nl.mathworks.com/help/matlab/ref/numargumentsfromsubscript.html 
        function n = numArgumentsFromSubscript(obj,s,~)
            if length(s) > 1 && strcmp(s(1).subs,'matrices') && s(2).subs{1} == ':'
                n = obj.Ng;
            else
                n = 1;
            end
        end
    end

    methods (Access='private')
        % Default method for one function arguments
        function A = oneArgFun(A, fun)
            A.matrices = cellfun(fun, A.matrices,'UniformOutput',false);
            A = simplify(A);
        end
    
        % Default method for two function arguments
        function A = twoArgFun(A, B, fun)
            if isa(A,'double')
                assert(numel(size(A)) <= 2);

                nGrid = B.Ng;
                mat = cell(nGrid,1);

                for i = 1:nGrid
                    mat{i} = fun(A, B.matrices{i});
                end
                A = B;  % make A a gridvar
            elseif isa(B,'double')
                assert(numel(size(B)) <= 2);

                nGrid = A.Ng;
                mat = cell(nGrid,1);

                for i = 1:nGrid
                    mat{i} = fun(A.matrices{i}, B);
                end
            else
                [A,B] = commongrid_(A,B);
                mat = cellfun(fun, A.matrices, B.matrices,'UniformOutput',false);
            end

            A.matrices = mat;
            A = simplify(A);
        end

        function varargout = commongrid_(varargin)
            varargout = varargin;

            dims = cell(nargin, 1);
            Ngs = zeros(nargin, 1);
            mainDim = 1;

            for i = 1:nargin
                varargout{i} = gridvar(varargin{i});
                dims{i} = varargout{i}.dim;
                Ngs(i) = varargout{i}.Ng;

                if dims{i} ~= 1
                    mainDim = dims{i};
                end
            end

            for i = 1:nargin
                assert((numel(dims{i}) == numel(mainDim) && all(dims{i} == mainDim)) || dims{i} == 1, 'Matrices have incompatible grids.');
            end

            ind = find(Ngs == 1);
            [mainNg] = max(Ngs);

            for i = 1:numel(ind)
                varargout{ind(i)}.matrices = repmat(varargout{ind(i)}.matrices,mainNg,1);
                varargout{ind(i)}.Ng = mainNg;
                varargout{ind(i)}.dim = mainDim;
            end
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

        function A = cat_(dim, A, B)
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
            if isempty(A)
                A = B;
                return;
            end
            if isempty(B)
                return;
            end
            A.matrices = cellfun(@(A,B)cat(dim,A,B),A.matrices,B.matrices,'UniformOutput',false);
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
            C = [A, zeros(nA, mB); ...
                zeros(nB, mA), B];
        end
        
    end
end

