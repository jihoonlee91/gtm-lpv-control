classdef timemap
    %TIMEMAP Maps scheduling signal to extended scheduling signal
    %
    %   Maps scheduling vector p(t) to extended (ext.) scheduling vector
    %   rho(t). Each element of rho(t) corresponds to a time-shifted (DT)
    %   or differentiated (CT) element of p(t).
    %
    %   See also PMATRIX
    
    properties (Dependent)
        % The Map is a numeric row matrix containing integers. In the DT
        % case, these integers represent the time-shifts that are used to
        % map the scheduling vector to the extended scheduling vector. In
        % the CT case, these (non-negative) integers represent the order of
        % the differentiation that appears in the extended scheduling
        % vector.
        %
        % Each element of Map is applied to ALL the elements of the
        % scheduling signal. For example, if the scheduling signal contains
        % two elements 'p' and 'q', then a Map [0, 1] corresponds to an
        % ext. scheduling signal of 4 elements (2 for 'p' and 2 for 'q').
        %
        % Examples for a scalar scheduling signal 'p':
        %
        %   [0, 1] in the DT case generates two ext. scheduling signals: 
        %   rho1(t) = p(t) and rho2(t) = p(t+1).
        %
        %   [0, 1] in the CT case also generates two ext. scheduling signals:
        %   rho1(t) = p(t) and rho2(t) = (dp/dt)(t).
        %
        Map
        
        % Character vector denoting the time domain of this TIMEMAP object.
        % Possible values are 'ct' (CT), 'dt' (DT), and 'undefined'. The
        % latter domain indicates that this timemap object can either
        % correspond to a CT or DT mapping. Timemaps of different domains
        % can only be merged if either of the domains is 'undefined'. 
        % Therefore, this is convenient when working
        % with timemaps for which the mapping is not (yet) associated to a
        % time domain.
        Domain
        
        % Cell array of character vectors representing the name of the
        % elements of the scheduling signal. The elements must have unique
        % names. In case a scalar name is passed for a non-scalar
        % scheduling signal, the scalar name is expanded according to the
        % following example:
        %
        %   {'p'} --> {'p(1)'; 'p(2)'; 'p(3)'}
        %
        % The default naming scheme for a scheduling signal with n elements 
        % is {'p1'; ...; 'pn'}.
        Name
        
        % Cell array of character vectors representing the units of the
        % element of the scheduling signal.
        %
        % Example: {'cm', 'kg', '-'}
        Unit
        
        % Cell array of ranges of values that each element in the
        % scheduling signal can assume. Each range is specified as a
        % numeric vector with 2 elements ([min, max]): a minimum and a maximum value.
        %
        % The range is NOT enforced when the timemap is evaluated. It is up
        % to the user to check that any input data respects these
        % constraints.
        %
        % Example: {[-1, +1], ..., [-Inf, +Inf]}
        Range
        
        % Cell array of bounds on the rate of change for each element in
        % the scheduling signal. Each bound is specified as a numeric
        % vector with 2 elements ([min, max]): a minimum and a maximum rate
        % of change.
        %
        % The rate bounds are NOT enforced when the timemap is evaluated.
        % It is up to the user to check that any input data respects these
        % constraints.
        %
        % The rate bounds apply to element of the scheduling signal only. 
        % Currently, it is not possible to set rate bounds for general
        % elements of the extended scheduling signal.
        %
        % Example: {[-1, +1], ..., [-Inf, +Inf]}
        RateBound
        
        % Cell array of character vectors corresponding to the names of the
        % extended scheduling signal. These names are automatically
        % generated from the Name and Map properties. Therefore, this
        % property is read-only.
        %
        % Example: when Name = {'p'} and Map = 0:
        %
        %   RhoName = {'p(t)'}
        %
        % Example: when Name = {'p', 'q'} and Map = [0, 1] (DT):
        %
        %   RhoName = {'p(t)', 'q(t)', 'p(t+1)', 'q(t+1)'}
        RhoName
        
        % Dimension of the scheduling signal.
        Dimension
    end
    
    properties (Hidden)
        Map_
        Domain_
        Name_
        Unit_
        Range_
        RateBound_
        RhoName_
        Dimension_
    end
    
    properties (Dependent, Hidden)
        np  % dimension of scheduling variable
        nrho  % dimension of ext. scheduling variable
    end
    
    methods
        function obj = timemap(varargin)
            %TIMEMAP Construct an instance of this class
            %
            %   Syntax:
            %       tm = timemap(map, domain)
            %
            %   Inputs:
            %       map (vector): vector of shifted versions of p(t) of
            %           which rho(t) is constructed. For example, if map = [0,
            %           1], then rho(t) = [p(t), p(t-1)] (DT) or rho(t) =
            %           [p(t), dp(t)/dt] (CT).
            %       domain: 'ct', 'dt' or 'undefined'.
            %       'Name': cell array of names for the elements of p(t).
            %       'Unit': cell array of names for the elements of p(t).
            %       'Range': cell array. Each entry is a vector of 2
            %           elements: the minimum and maximum value for the
            %           corresponding element of p(t).
            %       'RateBound': cell array. Each entry is a vector of 2
            %           elements: the minimum and maximum value for the rate of
            %           change of p(t).
            %
            p = inputParser;
            addOptional(p, 'map', []);
            addOptional(p, 'domain', 'ct', ...
                @(x) any(strcmpi(x, {'undefined', 'ct', 'dt'})));
            addParameter(p, 'Name', {});
            addParameter(p, 'Unit', {});
            addParameter(p, 'Range', {});
            addParameter(p, 'RateBound', {});
            addParameter(p, 'Dimension', 0, @(x) isint(x) && x >= 0);
            parse(p, varargin{:});
            obj.Domain_ = lower(p.Results.domain);
            % Infer Dimension from input arguments
            dimensionSpecified = ~any(strcmp('Dimension', p.UsingDefaults));
            if isempty(p.Results.Name) && ...
                    isempty(p.Results.Unit) && ...
                    isempty(p.Results.Range) && ...
                    isempty(p.Results.RateBound)
                if ~isempty(p.Results.map) && ~dimensionSpecified
                    dim = 1;  % override default dimension of 0 if a non-empty Map is passed
                else
                    dim = p.Results.Dimension;
                end
            else
                dim = max([numel(p.Results.Name), ...
                    numel(p.Results.Unit), numel(p.Results.Range), ...
                    numel(p.Results.RateBound)]);
                % Ensure no conflicting Dimension parameter was passed
                if dimensionSpecified && p.Results.Dimension ~= dim
                    error(['Dimension parameter does not match ', ...
                        'size of Name/Unit/Range/RateBound.']);
                end
            end
            % Initialize obj.Name_ so that obj.np returns correct dimension
            obj.Name_ = cell(dim, 1);
            % Initialize properties if empty
            if isempty(p.Results.Name)
                obj.Name_ = {};
                % If np > 1, add indices to p
                if dim > 1
                    for i=1:dim  % dim is needed since np doesn't exist yet
                        obj.Name_{i} = ['p', num2str(i)];
                    end
                elseif dim == 1
                    obj.Name_{1} = 'p';
                end
            else
                obj.Name = p.Results.Name;
            end
            if isempty(p.Results.Unit)
                obj.Unit_ = {};
                for i=1:obj.np
                    obj.Unit_{i} = '-';
                end
            else
                obj.Unit = p.Results.Unit;
            end
            if isempty(p.Results.Range)
                obj.Range_ = {};
                for i=1:obj.np
                    obj.Range_{i} = [-Inf, +Inf];
                end
            else
                obj.Range = p.Results.Range;
            end
            if isempty(p.Results.RateBound)
                obj.RateBound_ = {};
                for i=1:obj.np
                    obj.RateBound_{i} = [-Inf, +Inf];
                end
            else
                obj.RateBound = p.Results.RateBound;
            end
            % Build textual representation of rho(t) in terms of p(t)
            obj.Map = unique(p.Results.map, 'sorted');
            lint_(obj);
        end
        
        function v = isempty(obj)
            % ISEMPTY Returns whether timemap represents an empty map
            v = isempty(obj.Map);
        end
        
        function [C, idxRhoA, idxRhoB] = union(A, B)
            %UNION Takes union of two timemaps.
            %   Takes two timemaps and returns a third timemap that
            %   has an ext. scheduling parameter which is the union of
            %   the ext. scheduling parameters of the input timemaps.
            %
            %   Syntax:
            %       [C, idxA, idxB] = union(A, B)
            %
            %   Inputs:
            %       A, B (timemap): timemaps to join
            %
            %   Outputs:
            %       C (timemap): timemap constructed from A and B
            %       idxA, idxB (column vector): vectors with the same size
            %           as the map of A and B, respectively. Each element is the index
            %           of the new position in the map in C.
            %
            %   Example:
            %       >> A = timemap(0, 'dt')
            %       >> B = timemap(1, 'dt')
            %       >> [C, idxA, idxB] = union(A, B)
            %       idxA: [1, 0]'
            %       idxB: [0, 1]'
            %
            domain_ = timemap.unionDomain(A.Domain, B.Domain);
            if ~domain_
                error('Cannot merge timemaps of incompatible domains');
            end
            map_ = union(A.Map, B.Map, 'sorted');

            % Join the names, units, ranges and rate bounds
            [namesUnion, idxUnionA, idxUnionB] = union(A.Name, B.Name, 'stable');
            
            unitsUnion = {A.Unit{idxUnionA}, B.Unit{idxUnionB}};
            rangesUnion = {A.Range{idxUnionA}, B.Range{idxUnionB}};
            rateBoundsUnion = {A.RateBound{idxUnionA}, B.RateBound{idxUnionB}};
            [names_, I] = sort(namesUnion);
            units_ = unitsUnion(I);
            ranges_ = rangesUnion(I);
            rateBounds_ = rateBoundsUnion(I);
            
            idxNamesA = zeros(numel(A.Name), 1);
            for i=1:numel(idxNamesA)
                for j=1:numel(names_)
                    if strcmp(A.Name{i}, names_{j})
                        idxNamesA(i) = j;
                        break;
                    end
                end
            end
            
            idxNamesB = zeros(numel(B.Name), 1);
            for i=1:numel(idxNamesB)
                for j=1:numel(names_)
                    if strcmp(B.Name{i}, names_{j})
                        idxNamesB(i) = j;
                        break;
                    end
                end
            end
            
            idxMapA = zeros(numel(A.Map), 1);
            for i=1:numel(idxMapA)
                for j=1:numel(map_)
                    if A.Map(i) == map_(j)
                        idxMapA(i) = j;
                        break;
                    end
                end
            end
            idxMapB = zeros(numel(B.Map), 1);
            for i=1:numel(idxMapB)
                for j=1:numel(map_)
                    if B.Map(i) == map_(j)
                        idxMapB(i) = j;
                        break;
                    end
                end
            end
            
            % Calculate reindexing of rho based on reindexing of:
            %   map & names
            idxRhoA = zeros(numel(idxMapA) * numel(idxNamesA), 1);
            idxRhoB = zeros(numel(idxMapB) * numel(idxNamesB), 1);
            k = 1;
            for i=1:numel(idxMapA)
                for j=1:numel(idxNamesA)
                    idxRhoA(k) = (idxMapA(i)-1) * numel(names_) + idxNamesA(j);
                    k = k + 1;
                end
            end
            k = 1;
            for i=1:numel(idxMapB)
                for j=1:numel(idxNamesB)
                    idxRhoB(k) = (idxMapB(i)-1) * numel(names_) + idxNamesB(j);
                    k = k + 1;
                end
            end
            
            C = timemap(map_, domain_, ...
                'Name', names_, ...
                'Unit', units_, ...
                'Range', ranges_, ...
                'RateBound', rateBounds_);
        end
        
        function rho = createRho(obj, p, p0, pf)
            %CREATERHO Compatibility version for old version of toolbox.
            %
            %   Syntax:
            %       rho = timemap.createRho(p)
            %       rho = timemap.createRho(p, p0)
            %       rho = timemap.createRho(p, p0, pf)
            %
            %   Contrary to EVAL, this function returns a value for rho for
            %   each row in p. If p0 / pf are not specified, then these
            %   initial and final values are set to 0.
            if strcmp(obj.Domain, 'ct')
                error('Only implemented for discrete-time.');
            end
            if obj.np == 0
                rho = NaN(size(p));
                return;
            end
            maxShift = max(0, max(obj.Map));
            minShift = -min(0, min(obj.Map));
            if (nargin <= 2)
                p0 = zeros(minShift, obj.np);
            end
            if (nargin <= 3)
                pf = zeros(maxShift, obj.np);
            end
            rho = eval(obj, [p0; p; pf]);
        end
        
        function rho = eval(obj, p, Ts)
            %EVAL Calculates extended scheduling signal for given
            %scheduling signal.
            %
            %   Syntax:
            %       rho = timemap.eval(p)
            %       rho = timemap.eval(p, Ts)
            %
            %   Inputs:
            %       p (matrix): N-by-np matrix. Each row corresponds to a
            %           timestep. For DT timemaps, the first min(map) rows, 
            %           and the last max(map) rows will not be output, as they are
            %           required as initial and final values for
            %           calculation of the extended scheduling signal.
            %       Ts (positive number): sampling time. Used for numerical
            %           calculation of the gradient for CT timemaps.
            %
            %   Outputs:
            %       rho (matrix): M-by-nrho matrix. Each row corresponds to
            %           a timestep. For CT timemaps, M equals N. For DT
            %           timemaps, M will be smaller than or equal to N,
            %           depending on the maximum magnitude of the time
            %           shift in the map.
            %
            if strcmp(obj.Domain, 'ct')
                if nargin == 2
                    warning('Sample time Ts not specified, assuming 1');
                    Ts = 1;
                end
                rho = cteval_(obj, p, Ts);
            else
                rho = dteval_(obj, p);
            end
        end
        
        function varargout = freeze(obj, varargin)
            % FREEZE Returns rho for frozen (=constant) p
            %
            %   Syntax:
            %       [rho1, ..., rhon] = freeze(obj, p1, ..., pn)
            %
            %   Inputs:
            %       p1, ..., pn: row vector of Np elements with values of the
            %       frozen scheduling parameter. If a scalar is passed, the
            %           value is repeated Np times.
            %       rho1, ..., rhon: row vector of Nrho elements corresponding
            %       to frozen values p1, ..., pn. For CT timemaps,
            %       derivatives of p are set to 0. For DT timemaps, shifted
            %       versions are all equal.
            
            % Input validation
            N = numel(varargin);
            for i=1:N
                if ~isnumeric(varargin{i}) ...
                        || (~isscalar(varargin{i}) && ~all(eq(size(varargin{i}), [1, obj.np])) ...
                        && ~(obj.np == 0 && isempty(varargin{i})))
                    error('Each input must be 1-by-Np or scalar');
                end
                % Repeat scalar input
                if isscalar(varargin{i})
                    varargin{i} = ones(1, obj.np) * varargin{i};
                end
            end
            
            varargout = cell(1, N);
            for i=1:N
                p = varargin{i};
                rho = NaN(1, obj.nrho);
                for j=1:numel(obj.Map)
                    shift = obj.Map(j);
                    if strcmp(obj.Domain, 'ct')
                        if shift == 0
                            rho(1, (j-1)*obj.np+1:j*obj.np) = p;
                        else
                            rho(1, (j-1)*obj.np+1:j*obj.np) = zeros(1, obj.np);
                        end
                    else
                        rho(1, (j-1)*obj.np+1:j*obj.np) = p;
                    end
                end
                varargout{i} = rho;
            end
        end
        
        function obj = shift(obj, shiftAmount)
            %SHIFT Shifts timemap by specified amount.
            %   For DT timemaps, this corresponds to adding a delay. For CT
            %   timemaps, this corresponds to taking the derivative. In the
            %   latter case, only positive shift amounts are accepted.
            %
            %   Inputs:
            %       shiftAmount (int): amount of shift. Must be positive
            %           for CT timemaps. Default: 1.
            %
            if nargin == 1
                shiftAmount = 1;
            end
            if strcmp(obj.Domain, 'ct') && any(shiftAmount < 0)
                error('Only positive shift amounts are supported for CT timemaps.');
            end
            obj.Map = obj.Map + shiftAmount;
        end
        
        function f = eq(A, B)
            if any(size(A.Map) ~= size(B.Map))
                f = false;
                return;
            end
            f = all(A.Map == B.Map) && ...
                strcmp(A.Domain, B.Domain) && ...
                A.np == B.np && ...
                A.nrho == B.nrho && ...
                isequaln(A.Name, B.Name) && ...
                isequaln(A.Range, B.Range) && ...
                isequaln(A.RateBound, B.RateBound);
        end
        
        function f = ne(A, B)
            f = ~eq(A, B);
        end
        
        function val = get.Dimension(obj)
            val = obj.np;
        end
        
        function val = get.np(obj)
            val = numel(obj.Name_);
        end
        
        function val = get.nrho(obj)
            val = numel(obj.Map_) * numel(obj.Name_);
        end
        
        function val = get.RhoName(obj)
            assert(~strcmpi(obj.Domain_, 'undefined'), ...
                'RhoName is only available if Domain is well-defined ', ...
                '(ct or dt)');
            val = makeRhoName_(obj);
        end
        
        function val = get.Name(obj); val = obj.Name_; end
        function val = get.Unit(obj); val = obj.Unit_; end
        function val = get.Range(obj); val = obj.Range_; end
        function val = get.RateBound(obj); val = obj.RateBound_; end
        function val = get.Map(obj); val = obj.Map_; end
        function val = get.Domain(obj); val = obj.Domain_; end
        
        function obj = set.Domain(obj, val)
            assert(any(strcmpi(val, {'undefined', 'ct', 'dt'})), ...
                'Domain must be a character vector, either undefined, ct or dt');
            obj.Domain_ = val;
            % Verify whether Map is still valid
            obj.Map = obj.Map_;
        end
        
        function obj = set.Name(obj, val)
            obj.Name_ = LPVcore.cellarrayProperty('Name', val, obj.np);
        end
        
        function obj = set.Unit(obj, val)
            obj.Unit_ = LPVcore.cellarrayProperty('Unit', val, obj.np);
        end
        
        function obj = set.Map(obj, val)
            if ~isNumArrayOf(val, @(x) isint(x))
                error('MATLAB:InputParser:ArgumentFailedValidation', ...
                    'Map must be column or row vector with integer entries');
            end
            if ~isrow(val) && ~iscolumn(val)
                error('MATLAB:InputParser:ArgumentFailedValidation', ...
                    'Map must be column or row vector');
            end
            if strcmpi(obj.Domain_, 'ct')
                if min(val) < 0
                    error('MATLAB:InputParser:ArgumentFailedValidation', ...
                        'Continuous-time Map must have non-negative entries');
                end
            end
            % Transpose to row vector
            if iscolumn(val); val = val.'; end
            obj.Map_ = val;
        end
        
        function obj = set.Range(obj, val)
            val = timemap.toValidRange(val, obj.np, 'Range');
            obj.Range_ = val;
        end
        
        function obj = set.RateBound(obj, val)
            val = timemap.toValidRange(val, obj.np, 'RateBound');
            obj.RateBound_ = val;
        end
        
        function disp(obj)
            if isempty(obj)
                fprintf('Empty extended scheduling map\n');
                return;
            end
            % Header
            if strcmpi(obj.Domain, 'ct')
                timeStr = 'Continuous-time extended scheduling map';
            elseif strcmpi(obj.Domain, 'dt')
                timeStr = 'Discrete-time extended scheduling map';
            else
                fprintf('Extended scheduling map with undefined time domain\n\n');
                fprintf('Set time domain to ''ct'' or ''dt'' for additional displayed information\n\n');
                return;
            end
            fprintf('%s\n\n', timeStr);
            % Describe each scheduling signal
            fprintf('Scheduling signals:\n\n');
            for i=1:numel(obj.Name)
                range = obj.Range{i};
                schedStr = [obj.Name{i}, ' [', obj.Unit{i}, '] - ' ...
                    'Range: [', num2str(range(1)), ', ', num2str(range(2)), ']'];
                fprintf('\t%s\n', schedStr);
            end
            fprintf('\n');
            % Scheduling signal in vector form
            vecSchedStr = ['[ ', obj.Name{1}, '(t)'];
            for i=2:numel(obj.Name)
                vecSchedStr = [vecSchedStr, '; ', obj.Name{i}, '(t)']; %#ok<AGROW>
            end
            vecSchedStr = [vecSchedStr, ' ]'];
            % Describe ext. scheduling signal
            vecExtSchedStr = ['[ ', obj.RhoName{1}];
            for i=2:numel(obj.RhoName)
                vecExtSchedStr = [vecExtSchedStr, '; ', obj.RhoName{i}]; %#ok<AGROW>
            end
            vecExtSchedStr = [vecExtSchedStr, ' ]'];
            % Display
            fprintf('Map:\n\n');
            fprintf('\t%s\n', vecSchedStr);
            fprintf('\n\t\t|\n\t\t| mapped to %d ext. sched. signals\n\t\tv\n\n', ...
                obj.nrho);
            fprintf('\t%s\n\n', vecExtSchedStr);
        end
    end
    
    methods (Hidden, Static)
        function val = toValidRange(val, np, name)
            % TOVALIDRANGE Returns processed Range property or NaN if invalid. 
            %
            % Note: can also be used for RateBound property.
            
            if ~iscell(val); val = {val}; end
            
            % Transpose to column
            if ~iscolumn(val); val = val'; end
            
            % Expand scalar
            if numel(val) == 1 && np > 1
                val(1:np, 1) = val;
            end
            
            % Check whether supplied value is a valid Range-type property
            isValid = iscell(val) && numel(val) == np && ...
                all(cellfun(@(x) ...
                    isreal(x) && ...  % Exclude [1i, 3i]
                    isnumeric(x) && ...  % Exclude ['a', 'b']
                    numel(x) == 2 && ...
                    isrow(x) && ...  % Exclude [0; 1]
                    ~isnan(x(2) - x(1)) && ... % Exclude NaNs or [Inf, Inf]
                    x(1) <= x(2), val));
                
            if ~isValid
                error('MATLAB:InputParser:ArgumentFailedValidation', ...
                    [name, ' must be a cell array of ', ...
                    int2str(np), ' elements. Each element ', ...
                    'must be of the form [min, max], with min >= max']);
            end
        end

        function u = unionDomain(domain1, domain2)
            % UNIONDOMAIN Return the union of two time domains
            %
            %   Syntax:
            %       u = unionDomain(domain1, domain2)
            %
            %   Inputs:
            %       domain1, domain2: character vector of domain. Either
            %       'ct', 'dt' or 'undefined'.
            %
            %   Outputs:
            %       u: character vector of domain of union, or false if
            %       domains are incompatible (e.g. 'ct' and 'dt')
            %
            if strcmpi(domain1, 'undefined')
                u = domain2;
            elseif strcmpi(domain2, 'undefined')
                u = domain1;
            elseif strcmpi(domain1, domain2)
                u = domain1;
            else
                u = false;
            end
        end
    end
    
    methods (Access = private)   
        function rhoName = makeRhoName_(obj)
            %MAKERHONAME_ Make array of names of the ext. scheduling
            %vector based on time-shift, domain and names of sch. vector.
            k = 1;
            rhoName = cell(1, obj.np * numel(obj.Map));
            for i=1:numel(obj.Map)
                shiftAmount = obj.Map(i);
                for j=1:obj.np
                    if strcmp(obj.Domain, 'dt')
                        if shiftAmount > 0
                            rhoName{k} = [obj.Name{j}, ...
                                '(t+', num2str(shiftAmount), ')'];
                        elseif shiftAmount < 0
                            rhoName{k} = [obj.Name{j}, ...
                                '(t', num2str(shiftAmount), ')'];
                        else
                            rhoName{k} = [obj.Name{j}, ...
                                '(t)'];
                        end
                    else
                        if shiftAmount == 0
                            rhoName{k} = [obj.Name{j}, ...
                                '(t)'];
                        else
                            if shiftAmount ~= 1
                                powerString = ['^', num2str(shiftAmount)];
                            else
                                powerString = '';
                            end
                            rhoName{k} = ['(d', obj.Name{j}, powerString, ...
                                '/dt', powerString, ')', '(t)'];
                        end
                    end
                    k = k +1;
                end
            end
        end
        
        function rho = dteval_(obj, P)
            %DTEVAL_ Returns extended scheduling parameter for given
            %   trajectory of p, for discrete-time time-map.
            if isempty(obj.Map)
                rho = P;
                return;
            end
            M_min = -min(min(obj.Map), 0);
            M_max = max(max(obj.Map), 0);
            N = size(P, 1);
            M = N - M_min - M_max;
            rho = NaN(M, obj.nrho);
            for j=1:numel(obj.Map)
                shift = obj.Map(j);
                rho(:, (j-1)*obj.np+1:j*obj.np) = ...
                    P(M_min+shift+1:M_min+shift+M, :);
            end
        end
        
        function rho = cteval_(obj, P, Ts)
            %CTEVAL_ Returns ext. scheduling parameter for given trajectory
            %of p, for continuous-time timemap. Uses GRADIENT function to
            %approximate higher-order derivatives.
            N = size(P, 1);
            rho = NaN(N, obj.nrho);
            for j=1:numel(obj.Map)
                diff = obj.Map(j);
                rho(:, (j-1)*obj.np+1:j*obj.np) = P;
                for i=1:diff
                    if obj.np > 1
                        [~, rho(:, (j-1)*obj.np+1:j*obj.np)] = ...
                            gradient(rho(:, (j-1)*obj.np+1:j*obj.np), Ts);
                    else
                        rho(:, (j-1)*obj.np+1:j*obj.np) = ...
                            gradient(rho(:, (j-1)*obj.np+1:j*obj.np), Ts);
                    end
                end
            end
        end
        
        function lint_(obj)
            if numel(unique(obj.Name)) ~= numel(obj.Name)
                error('Names must be unique.');
            end
            if numel(obj.Unit) ~= obj.np
                error('Name/Unit/Range/RateBound must be a cell array with Dimension elements');
            end
            if numel(obj.Range) ~= obj.np
                error('Name/Unit/Range/RateBound must be a cell array with Dimension elements');
            end
            if numel(obj.RateBound) ~= obj.np
                error('Name/Unit/Range/RateBound must be a cell array with Dimension elements');
            end
            if ~any(strcmpi(obj.Domain, {'ct', 'dt', 'undefined'}))
                error('Domain must be one of either "ct" or "dt"');
            end
        end
    end
end

