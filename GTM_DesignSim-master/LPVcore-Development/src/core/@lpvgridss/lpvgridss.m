classdef lpvgridss
    %LPVGRIDSS LPV gridded state-space representation
    %
    % Linear Parameter-Varying system representation specified in terms of
    % local (i.e., frozen) models at a grid of operating points (i.e.,
    % values for the scheduling signal). The input data for this type of
    % system representation is therefore a set of Linear Time-Invariant
    % (LTI) system models, specified in terms of a state-space array, 
    % and the corresponding scheduling signal grid for that
    % linearization.
    %
    % A global behavior is defined from the grid by using the LPVINTERPSS
    % function. Example with piecewise linear ('pwlinear') interpolation:
    %
    %   >> container = lpvgridss(ssarray, grid)
    %   >> globalmodel = lpvinterpss(container, 'pwlinear')
    %
    % To create plots of the system at the specified grid points, there is
    % no need to manually specify the grid:
    %   lpvsys = lpvgridss(sys, grid, __)
    %   bode(lpvsys)
    %   bodemag(lpvsys)
    %   sigma(lpvsys)
    %
    % Syntax:
    %   lpvsys = lpvgridss(sys, grid)
    %
    % Inputs:
    %   sys: (N+2)-dimensional state-space array, with N the number of
    %       scheduling signals on which the grid should be defined. The
    %       first two dimensions are used for the input-output channels.
    %       The order of the dimensions should match the order that the
    %       scheduling signal grid is specified in the grid input
    %       parameter.
    %   grid: structure array with the grid points. Each field contains the
    %       grid points for the scheduling signal with the name of the
    %       field. Example: struct('p', [0, 1], 'q', [0, 1]).
    %
    % See also LPVINTERPSS
    %
    
    properties (SetAccess = private)
        % Structure describing the sampling grid used to create this
        % object. Each field in the structure corresponds to a scheduling
        % signal, the value being the grid points for that scheduling
        % signal.
        %
        % Example:
        %   SamplingGrid = struct('p', [0, 1], 'q', [0, 1])
        SamplingGrid
        % Model array containing the Linear Time-Invariant (LTI) models
        % corresponding to each point in SamplingGrid. The first 2
        % dimensions of ModelArray refer to the output and input channels,
        % respectively. Each subsequent dimension refers to a field in the
        % SamplingGrid structure array.
        ModelArray
        % Number of inputs
        Nu
        % Number of outputs
        Ny
        % Number of scheduling signals
        Np
        % Number of states
        Nx
        % Sampling time
        Ts
        % Unit of sampling time
        TimeUnit
    end

    properties (Dependent)
        % A-matrix of model at each grid point. Shorthand for
        % obj.ModelArray.A
        A
        % B-matrix of model at each grid point. Shorthand for
        % obj.ModelArray.B
        B
        % C-matrix of model at each grid point. Shorthand for
        % obj.ModelArray.C
        C
        % D-matrix of model at each grid point. Shorthand for
        % obj.ModelArray.D
        D
        % Input name
        InputName
        % Output name
        OutputName
        % Input unit
        InputUnit
        % Output unit
        OutputUnit
        % State unit
        StateUnit
        % Input delay for each input channel
        InputDelay
        % Output delay for each output channel
        OutputDelay
        % System name
        Name
        % Any text that you want to associate with the system
        Notes
        % Any type of data you want to associate with the system
        UserData
    end

    methods
        function obj = lpvgridss(sys, grid)
            %LPVGRIDSS Construct an instance of this class
            
            %% Input validation
            narginchk(2, 2);
            assert(isa(sys, 'ss'), '''sys'' must be a state-space model array');
            assert(size(sys, 1) > 0 && size(sys, 2), ...
                '''sys'' cannot have empty input or output');
            sz = size(sys);
            assert(numel(sz) >= 3, '''sys'' must be a state-space model array with at least 3 dimensions');
            % Truncate trailing 1s in sz, which MATLAB automatically adds
            % when creating a 1D state-space model array.
            if numel(sz) > 3
                for i=numel(sz):-1:4
                    if sz(i) == 1
                        sz(i) = [];
                    else
                        break;
                    end
                end
            end
            % Ensure each dimension of sys (after squeezing a potential last dimension of size 1)
            % has at least 2 elements
            assert(min(sz(3:end) > 1), 'Each scheduling signal must have at least 2 grid points');
            % Number of state-space models in array
            N = prod(sz(3:end));
            % Verify all state-space models have compatible state order and
            % sampling time
            nx = size(sys(:, :, 1).A, 1);
            Ts = sys(:, :, 1).Ts;
            for i=1:N
                sysi = sys(:, :, i);
                assert(nx == size(sysi.A, 1), ...
                    'All state-space models of ''sys'' must have identical state order');
                assert(Ts == sysi.Ts, ...
                    'All state-space models of ''sys'' must have identical sampling time Ts');
                assert(all(sysi.OutputDelay == 0), ...
                    'Non-zero OutputDelay not supported for ''sys''');
            end
            assert(isstruct(grid), '''grid'' must be a structure array');
            assert(numel(fieldnames(grid)) == numel(sz) - 2, ...
                'Number of fields in ''grid'' must equal number of state-space array dimensions minus two');
            f = fieldnames(grid);
            % gridCell is required for PBINTERP
            gridCell = cell(1, numel(f));
            for i=1:numel(f)
                assert(isnumeric(grid.(f{i})) && numel(grid.(f{i})) == sz(i+2), ...
                    [f{i}, ' in ''grid'' is not compatible with size of ''sys''']);
                gridCell{i} = grid.(f{i});
            end

            V = zeros(size(grid.(f{1}),2), 1);
            for i=2:numel(f)
                V = V .* zeros([ones(1, i-1) size(grid.(f{i}),2)]);
            end
            assert(numel(V) == N, 'LPVcore:gridds:ConstructorArgin',...
                    'number of systems doesn''t fit the grid.')
 
            obj.SamplingGrid = grid;
            obj.ModelArray = sys;
        end
        
        function bode(obj, varargin)
            sys = obj.ModelArray;
            bode(sys, varargin{:});
        end
        
        function bodemag(obj, varargin)
            sys = obj.ModelArray;
            bodemag(sys, varargin{:});
        end
        
        function sigma(obj, varargin)
            sys = obj.ModelArray;
            sigma(sys, varargin{:});
        end

        function val = get.Nu(obj)
            val = size(obj.ModelArray, 2);
        end

        function val = get.Ny(obj)
            val = size(obj.ModelArray, 1);
        end

        function val = get.Np(obj)
            val = numel(fieldnames(obj.SamplingGrid));
        end

        % Properties to inherit from ModelArray

        function val = get.Nx(obj)
            val = size(obj.A, 1);
        end

        function val = get.A(obj)
            val = obj.ModelArray.A;
        end

        function obj = set.A(obj,val)
            obj.ModelArray.A = val;
        end

        function val = get.B(obj)
            val = obj.ModelArray.B;
        end

        function obj = set.B(obj,val)
            obj.ModelArray.B = val;
        end

        function val = get.C(obj)
            val = obj.ModelArray.C;
        end

        function obj = set.C(obj,val)
            obj.ModelArray.C = val;
        end

        function val = get.D(obj)
            val = obj.ModelArray.D;
        end

        function obj = set.D(obj,val)
            obj.ModelArray.D = val;
        end

        function val = isct(obj)
            val = isct(obj.ModelArray);
        end

        function val = isdt(obj)
            val = isdt(obj.ModelArray);
        end

        function val = get.Ts(obj)
            val = obj.ModelArray.Ts;
        end

        function val = get.TimeUnit(obj)
            val = obj.ModelArray.TimeUnit;
        end

        function val = get.InputName(obj)
            val = obj.ModelArray.InputName;
        end

        function obj = set.InputName(obj, val)
            obj.ModelArray.InputName = val;
        end

        function val = get.OutputName(obj)
            val = obj.ModelArray.OutputName;
        end

        function obj = set.OutputName(obj, val)
            obj.ModelArray.OutputName = val;
        end

        function val = get.InputUnit(obj)
            val = obj.ModelArray.InputUnit;
        end

        function obj = set.InputUnit(obj, val)
            obj.ModelArray.InputUnit = val;
        end

        function val = get.OutputUnit(obj)
            val = obj.ModelArray.OutputUnit;
        end

        function obj = set.OutputUnit(obj, val)
            obj.ModelArray.OutputUnit = val;
        end

        function val = get.StateUnit(obj)
            val = obj.ModelArray.StateUnit;
        end

        function obj = set.StateUnit(obj, val)
            obj.ModelArray.StateUnit = val;
        end

        function val = get.InputDelay(obj)
            val = obj.ModelArray.InputDelay;
        end

        function obj = set.InputDelay(obj, val)
            obj.ModelArray.InputDelay = val;
        end

        function val = get.OutputDelay(obj)
            val = obj.ModelArray.OutputDelay;
        end

        function obj = set.OutputDelay(obj, val)
            obj.ModelArray.OutputDelay = val;
        end

        function val = get.Name(obj)
            val = obj.ModelArray.Name;
        end

        function obj = set.Name(obj, val)
            obj.ModelArray.Name = val;
        end

        function val = get.Notes(obj)
            val = obj.ModelArray.Notes;
        end

        function obj = set.Notes(obj, val)
            obj.ModelArray.Notes = val;
        end

        function val = get.UserData(obj)
            val = obj.ModelArray.UserData;
        end

        function obj = set.UserData(obj, val)
            obj.ModelArray.UserData = val;
        end

        % Interconnect functions

        function sys = horzcat(varargin)
            assertequalgrid(varargin{:});
            models = cellfun(@(x) x.ModelArray, varargin, 'UniformOutput', false);
            modelArray = horzcat(models{:});
            sys = lpvgridss(modelArray, varargin{1}.SamplingGrid);
        end

        function sys = vertcat(varargin)
            assertequalgrid(varargin{:});
            models = cellfun(@(x) x.ModelArray, varargin, 'UniformOutput', false);
            modelArray = vertcat(models{:});
            sys = lpvgridss(modelArray, varargin{1}.SamplingGrid);
        end

        function sys = blkdiag(varargin)
            assertequalgrid(varargin{:});
            models = cellfun(@(x) x.ModelArray, varargin, 'UniformOutput', false);
            modelArray = blkdiag(models{:});
            sys = lpvgridss(modelArray, varargin{1}.SamplingGrid);
        end

        function sys = append(varargin)
            sys = blkdiag(varargin{:});
        end

        function sys1 = series(sys1, sys2, varargin)
            assertequalgrid(sys1, sys2);
            sys1.ModelArray = series(sys1.ModelArray, sys2.ModelArray, varargin{:});
        end

        function sys1 = parallel(sys1, sys2, varargin)
            assertequalgrid(sys1, sys2);
            sys1.ModelArray = parallel(sys1.ModelArray, sys2.ModelArray, varargin{:});
        end

        function sys1 = feedback(sys1, sys2, varargin)
            assertequalgrid(sys1, sys2);
            sys1.ModelArray = feedback(sys1.ModelArray, sys2.ModelArray, varargin{:});
        end

        function sys = ctranspose(sys)
            sys.ModelArray = ctranspose(sys.ModelArray);
        end

        function sys = transpose(sys)
            sys = transpose(sys.ModelArray);
        end

        function sys1 = lft(sys1, sys2, varargin)
            assertequalgrid(sys1, sys2);
            sys1.ModelArray = lft(sys1.ModelArray, sys2.ModelArray, varargin{:});
        end

        function sysc = connect(varargin)
            % Check which input arguments are LPVGRIDSS objects. From
            % these, ModelArray, should be extracted and passed to built-in
            % CONNECT.
            gridssArgsLogicalIndex = cellfun(@(x) isa(x,'lpvgridss'), varargin);
            gridssIndex = find(gridssArgsLogicalIndex);
            sys = varargin(gridssArgsLogicalIndex);
            num_sys = numel(sys);
            sysModelArray = cell(1,num_sys);
            for i=1:num_sys
                sysModelArray{i} = varargin{gridssIndex(i)}.ModelArray;
            end
            assertequalgrid(sys{:});
            nonGridssArgs = varargin(~gridssArgsLogicalIndex);
            sysc = sys{1};
            sysc.ModelArray = connect(sysModelArray{:}, nonGridssArgs{:});
        end

        function sz = size(obj)
            % SIZE Get size of gridded LPV-SS representation
            %
            % Syntax:
            %   sz = size(obj)
            %
            % Outputs:
            %   sz: row vector of integers. The first two integers are the
            %   output and input size, respectively. The remaining integers
            %   each represent the number of grid points corresponding to
            %   the scheduling signal in obj.SamplingGrid.
            %
            % Example:
            %   sz = size(obj) --> sz = [2, 3, 4, 5]
            %   Indicates obj has 2 outputs, 3 inputs, 4 grid points for
            %   the 1st scheduling signal and 5 grid points for the 2nd
            %   scheduling signal.
            %
            narginchk(1, 1);
            sz = size(obj.ModelArray);
            % Truncate trailing 1s in sz, which MATLAB automatically adds
            % when creating a 1D state-space model array.
            if numel(sz) > 3
                for i=numel(sz):-1:4
                    if sz(i) == 1
                        sz(i) = [];
                    else
                        break;
                    end
                end
            end
        end
    end

    methods (Hidden)
        function assertequalgrid(varargin)
            % ASSERTEQUALGRID Asserts that sampling grid of all input models
            % are equal
            %
            %   Syntax:
            %       tf = assertequalgrid(sys1, sys2, ..., sysn)
            %
            if nargin == 1
                return
            end
            grids = cellfun(@(x) x.SamplingGrid, varargin, 'UniformOutput', false);
            assert(isequal(grids{:}), '''SamplingGrid'' of all models must be equal');
        end
    end
end
        