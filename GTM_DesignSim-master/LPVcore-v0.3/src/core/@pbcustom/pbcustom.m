classdef pbcustom < pbasis
    %PBCUSTOM Custom basis function
    %   This basis function is parametrized by a function handle.
    %   The names of the variables in the function handle must be of the
    %   following form:
    %
    %       @(rho) f( rho(:, 1), ..., rho(:, N) )
    %
    %   The colons are required to ensure the basis function can use
    %   vectorization.
    %
    
    properties (SetAccess = 'private')
        f   % function handle
    end
    
    methods
        function obj = pbcustom(f)
            %PBCUSTOM Construct an instance of this class
            if isa(f, 'pbcustom')
                obj = f;
                return;
            elseif isa(f, 'pbpoly')
                s = ['@(rho)', str(f)];
            elseif isa(f, 'double')
                assert(isscalar(f), ...
                    'First input must be a scalar number or function handle');
                s = ['@(rho)', num2str(f)];
            else
                assert(isa(f, 'function_handle'), ...
                    'First input must be a scalar number or function handle');
                s = func2str(f);
                old = '(:, ';
                new = '(:,';
                s = strrep(s, old, new);
            end
            
            % Function handle must start with @(rho)
            if ~startsWith(s, '@(rho)')
                error('Custom function handle must start with "@(rho)".');
            end
            % Each occurrence of rho must be vectorized ("rho(:, x)")
            if count(s, 'rho') ~= (count(s, 'rho(') + 1) || ...
                    count(s, 'rho(') ~= count(s, 'rho(:,')
                error(['Each occurrence in custom function handle must ', ...
                    'be vectorized (i.e., "rho(:, x)")']);
            end
            f = str2func(s);
            obj.f = f;
            obj.sortKey = string(extractAfter(s,'@(rho)'));%MATLAB '2str' usually is a character array.
            obj.dependenceType = 'custom';
        end
        
        function C = times(A, B)
            A = pbcustom(A);
            B = pbcustom(B);
            f1 = A.f; %TODO: replace following 6 lines with obj.sortKey
            f2 = B.f;
            s1 = func2str(f1);
            s2 = func2str(f2);
            old = '@(rho)';
            new = '';
            s1 = strrep(s1, old, new);
            s2 = strrep(s2, old, new);
            s3 = ['@(rho) (', s1, ') * (', s2, ')'];
            f3 = str2func(s3);
            C = pbcustom(f3);
        end
        
        function f = feval(obj, rho)
            f = obj.f(rho);
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
            s = func2str(obj.f);
            
            for i=1:numel(I)
                old = ['(:,', int2str(i), ')'];
                new = ['(:,', int2str(I(i)), '_tmp)'];
                s = strrep(s, old, new);
            end
            
            for i=1:numel(I)
                old = ['(:,', int2str(I(i)), '_tmp)'];
                new = ['(:,', int2str(I(i)), ')'];
                s = strrep(s, old, new);
            end
            
            obj.f = str2func(s);
            obj.sortKey = string(extractAfter(s,'@(rho)'));
        end
        
        function s = str(obj, rhoNames)
            s = char(obj.sortKey);
            % Replace occurrences of "rho(:, x)" by actual rhoName if
            % provided
            if nargin > 1
                for i=1:numel(rhoNames)
                    sOld = ['rho(:,', int2str(i), ')'];
                    sNew = rhoNames{i};
                    s = replace(s, sOld, sNew);
                end
            end
        end
        
        function minNrho = minNRho(obj, varargin)
            if nargin > 1
                rho = minNRho(varargin{:});
            else
                rho = 0;
            end
            func = char(obj.sortKey);
            before = strfind(func, 'rho(:,');
            for ii = before
                after = strfind(func(ii+6 : end), ')');
                rho = [rho str2double(string(func((ii+6):after(1)+ii+4)))]; %#ok<AGROW>
            end
            minNrho = max(rho);
        end
    end
    
    methods (Hidden, Access=protected)
        function diff_(~, ~)
            error('Not implemented yet');
        end
    end
end

