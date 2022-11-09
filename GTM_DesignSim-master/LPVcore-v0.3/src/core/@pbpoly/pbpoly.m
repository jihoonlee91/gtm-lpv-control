classdef pbpoly < pbasis
    %PBPOLY Polynomial basis function
    %   This basis function models a polynomial.
    %   It is parametrized by a matrix "degrees" and a vector "coeff". Each
    %   row vector of "degrees" specifies a single monomial, with the degree for each
    %   element of the extended scheduling vector. For example:
    %
    %       degrees = [1, 2, 0, 3]
    %       coeff = 1
    %
    %   corresponds to the following monomial:
    %
    %       rho1 * rho2^2 * 1 * rho4^3
    %
    %   And:
    %
    %       degrees = [1, 2; 0, 3]
    %       coeff = [1, 2]
    %
    %   corresponds to the following polynomial:
    %
    %       1 * rho1 * rho2^2 + 2 * rho2^3
    %
    
    properties
        degrees % matrix of degrees for each monomial (rows) and element of the extended scheduling vector (columns)
        coeff % vector of coefficients for each monomial
    end
    
    methods
        function obj = pbpoly(degrees, coeff)
            %PBPOLY Construct an instance of this class
            %
            %   Syntax:
            %       p = pbpoly(degrees)
            %
            %   Inputs:
            %       degrees (matrix of integers): degree of each element of rho.
            %
            obj.degrees = degrees;
            if nargin <= 1
                coeff = ones(1, size(degrees, 1));
            end
            obj.coeff = coeff;
            obj = unique_(obj);
            obj.dependenceType = 'polynomial';
        end
        
        function obj = reidx(obj, I)
            %REIDX Reconstructs basis function after transforming rho.
            %   When two pmatrix objects are joined, the ext. scheduling
            %   parameter rho will be augmented with the union of the
            %   rho vectors of the input pmatrix objects. This change
            %   should be propagated to the basis fucntions. REIDX takes
            %   care of reindexing the expression.
            %
            %   Syntax:
            %       p = p.reidx(I)
            %
            %   Inputs:
            %       I (column vector): vector with the samen number of
            %       elements as the old ext. scheduling vector rho. Each
            %       element of I is the corresponding index of the new
            %       ext. scheduling vector.
            %
            %   Outputs:
            %       p: modified basis function
            %
            newDegrees = zeros(size(obj.degrees, 1), max(I));
            for i=1:size(obj.degrees, 2)
                newDegrees(:, I(i)) = obj.degrees(:, i);
            end
            obj.degrees = newDegrees;
            obj = unique_(obj);
        end
        
        function f = feval(obj, rho)
            %FEVAL Calculates value of basis function at given rho
            %
            %   Syntax:
            %       f = p.feval(rho)
            %
            %   Inputs:
            %       rho: N-by-nrho matrix
            %
            %   Outputs:
            %       f: column vector of size N
            %
            m = size(obj.degrees, 2);
            if size(rho, 2) < m
                error('rho too small for number of variables.');
            end
            % Sum over monomials
            f = 0;
            for i=1:size(obj.degrees, 1)
                % Evaluate each monomial
                f = f + obj.coeff(i) * prod(rho(:, 1:m) .^ (obj.degrees(i, :)), 2);
            end
        end
        
        function A = times(A, B)
            if isa(A, 'pbpoly') && isa(B, 'pbpoly')
                % Get sizes
                N_mA = numel(A.coeff);
                N_mB = numel(B.coeff);
                N_rhoA = size(A.degrees, 2);
                N_rhoB = size(B.degrees, 2);
                % Ensure the degrees matrices have the same number of
                % columns
                A.degrees = [A.degrees, zeros(N_mA, max(0, N_rhoB - N_rhoA))];
                B.degrees = [B.degrees, zeros(N_mB, max(0, N_rhoA - N_rhoB))];
                % Iterate over monomials
                degrees_ = NaN(N_mA * N_mB, max(N_rhoA, N_rhoB));
                coeff_ = NaN(N_mA * N_mB, 1);
                for i=1:N_mA
                    for j=1:N_mB
                        ind = sub2ind([N_mA, N_mB], i, j);
                        degrees_(ind, :) = A.degrees(i, :) + B.degrees(j, :);
                        coeff_(ind) = A.coeff(i) * B.coeff(j);
                    end
                end
                % Update degrees and coeff
                A.degrees = degrees_;
                A.coeff = coeff_;
                % Merge non-unique monomials
                A = unique_(A);
            else
                A = times(B, A);
            end
        end
        
        function s = str(obj, rhoNames)
            %STR Return string representation for display.
            s = '';
            if all(obj.degrees == 0)
                s = num2str(obj.coeff);
                return;
            end
            if nargin == 1
                m = numel(obj.degrees);
                rhoNames = cell(1, m);
                for i=1:m
                    rhoNames{i} = ['rho(:,', int2str(i), ')'];
                end
            end
            for j=1:size(obj.degrees, 1)
                c = obj.coeff(j);
                if all(obj.degrees(j,:) == 0)
                    s = [s, int2str(c)]; %#ok<AGROW>
                else
                    if j >= 2
                        if c > 0
                            s = [s, ' + ']; %#ok<AGROW>
                        else
                            s = [s, ' - ']; %#ok<AGROW>
                        end
                    end
                    if abs(c) ~= 1
                        s = [s, num2str(abs(c)), '*']; %#ok<AGROW>
                    end
                    % Flag to keep track of whether the term is the first that
                    % is display in this monomial (i.e., non-zero degree)
                    isFirstDisplayedInMonomial = true;
                    for i=1:size(obj.degrees, 2)
                        d = obj.degrees(j, i);
                        if d >= 1
                            if ~isFirstDisplayedInMonomial
                                s = [s, '*']; %#ok<AGROW>
                            end
                            isFirstDisplayedInMonomial = false;
                            s = [s, rhoNames{i}]; %#ok<AGROW>
                            if d >= 2
                                s = [s, '^', int2str(d)]; %#ok<AGROW>
                            end
                        end
                    end
                end
            end
        end
        
        function minNrho = minNRho(obj, varargin)

            m = size(obj.degrees, 2);
            if nargin > 1
                minNrho = max([m; minNRho(varargin{:})]);
            else
                minNrho = m;
            end

        end
    end
    
    methods (Hidden, Access=protected)
        function A = unique_(A)
            % UNIQUE_ Collapse monomials with identical degrees
            [degrees_, ~, ic] = unique(A.degrees, 'rows');
            % Add coefficients
            coeff_ = zeros(size(degrees_, 1), 1);
            for i=1:numel(ic)
                coeff_(ic(i)) = coeff_(ic(i)) + A.coeff(i);
            end
            % TODO: remove monomials with 0 coefficient

            %removing trailing zeros:
            index = find(degrees_);
            if size(index,2) > 0 && size(degrees_,1) == 1%check for not empty(pbconst) and dontcare for matrixes
                degrees_ = degrees_(1:index(end));
            end
            
            A.degrees = degrees_;
            A.coeff = coeff_;
            A.sortKey = string(str(A));
        end
        
        function A = diff_(A, Np)
            % DIFF_ Derivative of A w.r.t. time
            
            Nm = size(A.degrees, 1);
            Nrho = size(A.degrees, 2);
            
            degrees_ = NaN(0, Nrho + Np);
            coeff_ = NaN(0, 1);
            for i=1:Nm
                for j=1:Nrho
                    % Derivative w.r.t. j'th variable
                    degreesNew = zeros(1, Nrho + Np);
                    if A.degrees(i, j) > 0
                        degreesNew(1:Nrho) = A.degrees(i, :);
                        degreesNew(j) = degreesNew(j) - 1;
                        degreesNew = ...
                            [degreesNew, zeros(max(0, (j+Np) - numel(degreesNew)))]; %#ok<AGROW>
                        degreesNew(j+Np) = degreesNew(j+Np) + 1;
                        coeffNew = A.coeff(i) * A.degrees(i, j);
                        degrees_ = [degrees_; degreesNew]; %#ok<AGROW>
                        coeff_ = [coeff_; coeffNew]; %#ok<AGROW>
                    end
                end
            end
            A.degrees = degrees_;
            A.coeff = coeff_;
            A = unique_(A);
        end
    end
end

