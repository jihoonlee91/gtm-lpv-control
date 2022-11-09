% LPVLFR LPV model in LFR representation
%
%   This class models the following system interconnection:
%
%             +---------+
%             |         |
%       +-----+ Delta(p)+<---+
%       |     |         |    |
%       |     +---------+    |
%     w |                    | z
%       |   +------------+   |
%       |   |            |   |
%       +-->+     G      +---+
%           |            |
% u +------>+            +-------> y
%           +------------+
%
%
%   Delta(p): Nw-by-Nz PMATRIX object
%
%   G: LTI system with an SS representation partitioned as follows:
%
%       G = [A0, Bw, Bu; ...
%           Cz, Dzw, Dzu; ...
%           Cy, Dyw, Dyu]
%
%   The sample time of an LPVLFR model is inherited from G.
%
%   Syntax:
%       sys = lpvlfr(Delta, G)
%       sys = lpvlfr(Delta, G, Name, Value)
%
%   Inputs:
%       Delta: PMATRIX
%       G: LTI system (SS, TF)
%
%   Name, Value pairs:
%       InputName: name of each input channel
%       InputUnit: unit of each input channel
%       OutputName: name of each output channel
%       OutputUnit: unit of each output channel
%
classdef (InferiorClasses = {?ss, ?tf, ?zpk}) lpvlfr < lpvrep    
    properties (Dependent)
        G        % LTI state space model or transfer function
        Delta    % PMATRIX of the delta block
        Nx       % state dimension of G
        Nz       % input size of Delta
        Nw       % output size of Delta
        A0          % A0 matrix of state-space partitioning of G
        Bw          % Bw matrix of state-space partitioning of G
        Bu          % Bu matrix of state-space partitioning of G
        Cz          % Cz matrix of state-space partitioning of G
        Dzw         % Dzw matrix of state-space partitioning of G
        Dzu         % Dzu matrix of state-space partitioning of G
        Cy          % Cy matrix of state-space partitioning of G
        Dyw         % Dyw matrix of state-space partitioning of G
        Dyu         % Dyu matrix of state-space partitioning of G
        StateName   % Cell array of state names
        StateUnit   % Cell array of state units
    end
    
    properties (Hidden)
        G_
        Delta_
        StateName_ = {}
        StateUnit_ = {}
    end
    
    methods(Static)
        varargout = commonrho_(varargin)
    end
    
    methods
        function obj = lpvlfr(Delta, G, varargin)
            %LPVLFR Construct an instance of this class
            
            % check if delta is a pmatrix or empty
            if ~isa(Delta, 'pmatrix') && ~isempty(Delta)
                if isa(Delta, 'lpvlfr')
                    obj = lpvlfr(Delta.Delta, Delta.G);
                    obj = LPVcore.copySignalProperties(obj, Delta, 'full');
                    return;
                elseif isnumeric(Delta)
                    obj = lpvlfr(lpvss(Delta));
                    return;
                elseif isa(Delta, 'tf') || isa(Delta, 'ss') || isa(Delta, 'zpk')
                    obj = lpvlfr(lpvss(Delta));
                    return;
                end
                error('Delta must be a pmatrix, or empty');
            end
            G = ss(G);
            % If G is static, default sampling time is 0
            if isstatic(G) && ~isempty(Delta) && strcmp(Delta.timemap.Domain, 'dt')
                G.Ts = -1;
            end
            % Check time domain compatibility between G and Delta
            if ~isempty(Delta) && G.Ts == 0 && (~strcmp(Delta.timemap.Domain, 'ct'))
                error('G and Delta must both be continuous or discrete time.')
            end
            if ~isempty(Delta) && (G.Ts == -1 || G.Ts>0) && ( ~strcmp(Delta.timemap.Domain, 'dt'))
                error('G and Delta must both be continuous or discrete time.')
            end
            % assign local variables.
            obj.G_ = G;
            obj.Delta_ = obj2pmatrix(Delta);
            obj.Ts_ = G.Ts;
            if ~isempty(Delta)
                obj.SchedulingTimeMap = obj.Delta_.timemap;
            else
                if obj.Ts == -1 || obj.Ts > 0
                    obj.SchedulingTimeMap = timemap([], 'dt');
                else
                    obj.SchedulingTimeMap = timemap([], 'ct');
                end
            end
            % check inputs are positive
            if obj.Nu < 0 || obj.Ny < 0
                error('Delta block must be at most as large as G.');
            end
            % parse Name, Value pairs
            parser = inputParser;
            parser.KeepUnmatched = true;
            addParameter(parser, 'StateName', {});
            addParameter(parser, 'StateUnit', {});
            parse(parser, varargin{:});
            if ~isempty(parser.Results.StateName)
                obj.StateName = parser.Results.StateName;
            end
            if ~isempty(parser.Results.StateUnit)
                obj.StateUnit = parser.Results.StateUnit;
            end
            obj = parsePropertyValuePairs(obj, parser.Unmatched);
            % Set default properties
            if isempty(obj.StateName)
                obj.StateName(1:obj.Nx, 1) = {''};
            end
            if isempty(obj.StateUnit)
                obj.StateUnit(1:obj.Nx, 1) = {''};
            end
        end

        function [sysr, u] = minreal(sys, varargin)
            % MINREAL Minimal realization of LTI block
            %
            % Syntax:
            %   sysr = minreal(sys)
            %   sysr = minreal(sys, tol)
            %   [sysr, u] = minreal(__)
            %   ... = minreal(sys, tol, false)
            %   ... = minreal(sys, [], false)
            %
            % Inputs:
            %   sys: LPVLFR model
            %   tol: tolerance for state elimination
            %   false: disables verbose output
            %
            % Output:
            %   sysr: LPVLFR model with MINREAL called on the LTI block
            %   u: an orthogonal matrix such that (U*A*U', U*B, C*U') is a
            %       Kalman decomposition of (A,B,C), with (A, B, C) being the
            %       state-space matrices of the LTI block of sys (sys.G).
            %
            [Gr, u] = minreal(sys.G, varargin{:});
            sysr = sys;
            sysr.G = Gr;
        end
        
        function sysf = feedback(varargin)
            % FEEDBACK Feedback connection of multiple LPV-LFR models
            %
            %   Syntax:
            %       sys = feedback(sys1, sys2)
            %       sys = feedback(sys1, sys2, feedin, feedout)
            %       sys = feedback(sys1, sys2, 'name')
            %
            %   Description:
            %       The syntax is the same as FEEDBACK from the Control
            %       System Toolbox.
            %
            %   See also: CONNECT
            
            % check if there are atleast 2 inputs
            narginchk(2, Inf);
            
            % create blksystem and create connections using standard -1 sign
            blksys = append(varargin{1},varargin{2});
            
            % check what kind of feedback is required
            if length(varargin) == 2            % feedback(sys1,sys2)
                connections = zeros(blksys.Nu,2);
                for i=1:varargin{1}.Nu
                    connections(i,:) = [i -varargin{1}.Nu-i];
                end
                for i=(varargin{1}.Nu+1):blksys.Nu
                    connections(i,:) = [i i-varargin{1}.Nu];
                end
                sysf = connect(blksys,connections,1:varargin{1}.Nu, 1:varargin{1}.Ny);
            elseif length(varargin) == 3
                if isa(varargin{3}, 'char')  % feedback(sys1,sys2,'name')
                    sysf = connect(varargin{1},-varargin{2},varargin{1}.InputName,varargin{1}.OutputName);
                else                         % feedback(sys1,sys2, sign)
                    connections = zeros(blksys.Nu,2);
                    for i=1:varargin{1}.Nu
                        if varargin{3} == 1
                            connections(i,:) = [i abs(-varargin{1}.Nu-i)];
                        else
                            connections(i,:) = [i -varargin{1}.Nu-i];
                        end
                    end
                    for i=(varargin{1}.Nu+1):blksys.Nu
                        connections(i,:) = [i i-varargin{1}.Nu];
                    end
                    sysf = connect(blksys,connections,1:varargin{1}.Nu, 1:varargin{1}.Ny);
                end
            elseif length(varargin) == 4
                if isa(varargin{3}, 'char')  % feedback(sys1,sys2,'name', sign)
                    if varargin{4} == 1 % positive feedback
                        sysf = connect(varargin{1},varargin{2},varargin{1}.InputName,varargin{1}.OutputName);
                    else
                        sysf = connect(varargin{1},-varargin{2},varargin{1}.InputName,varargin{1}.OutputName);
                    end
                else                         % feedback(sys1,sys2,feedin,feedout)
                    connections = zeros(blksys.Nu-varargin{1}.Nu+length(varargin{3}),2);
                    for i=1:length(varargin{3})
                        connections(i,:) = [varargin{3}(i), -varargin{1}.Ny-i];
                    end
                    for i = 1:length(varargin{4})
                        connections(i+length(varargin{3}),:) = [i+varargin{1}.Ny, varargin{4}(i)];
                    end
                    sysf = connect(blksys,connections,1:varargin{1}.Nu, 1:varargin{1}.Ny);
                end
            else % feedback(sys1,sys2,feedin,feedout,sign)
                connections = zeros(blksys.Nu-varargin{1}.Nu+length(varargin{3}),2);
                for i=1:length(varargin{3})
                    if varargin{5} == 1
                        connections(i,:) = [varargin{3}(i), abs(-varargin{1}.Ny-i)];
                    end
                end
                for i = 1:length(varargin{4})
                    connections(i+length(varargin{3}),:) = [i+varargin{1}.Ny, varargin{4}(i)];
                end 
                sysf = connect(blksys,connections,1:varargin{1}.Nu, 1:varargin{1}.Ny);
            end
            
        end

        function sysc = connect(varargin)
            % CONNECT Block diagram interconnections of dynamic systems
            %
            %   Syntax:
            %       sysc = connect(blksys, connections, inputs, outputs)
            %
            %   Inputs:
            %       blksys: aggregated LPV-LFR system
            %       connections: matrix specifying the connections and
            %       summing junctions of the block diagram
            %       inputs: index vectors of inputs to keep
            %       outputs: index vectors of outputs to keep
            %
            %   Outputs:
            %       sysc: interconnected LPV-LFR system
            %
            %   Alternative usage:
            %
            %       sysc = connect(sys1, ..., sysN, inputs, outputs)
            %
            %   Connects systems sys1, ..., sysN based on signal names.
            %
            if nargin == 4 && isa(varargin{1}, 'lpvlfr') && isnumeric(varargin{2}) && ...
                    isnumeric(varargin{3}) && isnumeric(varargin{4})
                % Index based interconnection
                blksys = varargin{1};
                connections = varargin{2};
                inputs = varargin{3};
                outputs = varargin{4};
                sysc = connect_(blksys, connections);
                sysc = subsref_(sysc, outputs, inputs);
            else
                N = nargin - 2;
                sys = cell(1, N);
                for i=1:N
                    if ~isa(varargin{i}, 'lpvlfr')
                        sys{i} = lpvlfr(varargin{i});
                    else
                        sys{i} = varargin{i};
                    end
                end
                blksys = blkdiag(sys{:});
                % Convert to index-based problem
                inputName = blksys.InputName;
                outputName = blksys.OutputName;
                connections = [];
                % Expand vector-valued signal names (e.g., {'u'} --> {'u(1)', 'u(2)'}
                inputsByName = varargin{end-1};
                if ~iscell(inputsByName); inputsByName = {inputsByName}; end
                inputsByName = lpvrep.expandSignalNames_(inputsByName, inputName);
                outputsByName = varargin{end};
                if ~iscell(outputsByName); outputsByName = {outputsByName}; end
                outputsByName = lpvrep.expandSignalNames_(outputsByName, ...
                    unique([outputName; inputName]));
                % Loop over named inputs and convert to indices
                inputs = [];
                outputs = [];
                for i=1:numel(inputsByName)
                    if any(strcmp(inputsByName{i}, inputName))
                        inputs = [inputs, ...
                            find(strcmp(inputsByName{i}, inputName))]; %#ok<AGROW>
                    else
                        error(['The signal name ''', inputsByName{i}, ...
                        ''' is not listed as input by any block']);
                    end
                end
                % Loop over named outputs and convert to indices
                for i=1:numel(outputsByName)
                    if any(strcmp(outputsByName{i}, outputName))
                        outputs = [outputs, ...
                            find(strcmp(outputsByName{i}, outputName))]; %#ok<AGROW>
                    end
                end
                for i=1:numel(inputName)
                    input = inputName{i};
                    for j=1:numel(outputName)
                        output = outputName{j};
                        if strcmp(input, output)
                            connections = [connections; ...
                                i, j]; %#ok<AGROW>
                        end
                    end
                end
                % Solve index-based connection
                sysc = connect(blksys, connections, inputs, outputs);
                % Feed through named outputs that are inputs
                if numel(outputsByName) ~= numel(outputs)
                    for i=1:numel(outputsByName)
                        % Check if output is missing and add feedthrough
                        if ~strcmp(sysc.OutputName, outputsByName{i})
                            sysc = addFeedthrough_(sysc, outputsByName{i}, i);
                        end
                    end
                end
            end
        end
        
        function C = cat(dim, varargin)
            % CAT Concatenate LPV-LFR models
            %
            %   Syntax:
            %       sys = cat(dim, sys1, sys2, ...)
            %
            %   Inputs:
            %       dim: dimension along which to concatenate
            %       sys1, sys2, ...: LPV-LFR models to concatenate
            %
            %   Outputs:
            %       sys: concatenated LPV-LFR model
            %
            [varargin{1:nargin-1}] = lpvlfr.commonrho_(varargin{:});
            C = varargin{1};
            
            % iteratively concatenate pairs of lpvlfr objects
            for i=2:nargin-1
                C = cat_(dim, C, varargin{i});
            end
        end
        
        function C = vertcat(varargin)
            % VERTCAT Concatenate LPV-LFR models vertically
            %
            %   Syntax:
            %       sys = vertcat(sys1, ...)
            %
            %   Inputs:
            %       sys1, ...: LPV-LFR models to concatenate
            %
            %   Outputs:
            %       sys: concatenated LPV-LFR model
            %
            C = cat(1, varargin{:});
        end
        
        function C = horzcat(varargin)
            % HORZCAT Concatenate LPV-LFR models horizontally
            %
            %   Syntax:
            %       sys = horzcat(sys1, ...)
            %
            %   Inputs:
            %       sys1, ...: LPV-LFR models to concatenate
            %
            %   Outputs:
            %       sys: concatenated LPV-LFR model
            %
            C = cat(2, varargin{:});
        end
        
        function obj = uplus(obj)
            % UPLUS Unary addition
        end
        
        function obj = uminus(obj)
            % UMINUS Unary subtraction
            A = obj.A0;
            C = [obj.Cz; -obj.Cy];
            B = [obj.Bw, obj.Bu];
            D = [obj.Dzw, obj.Dzu; -obj.Dyw, -obj.Dyu];
            obj.G_ = ss(A, B, C, D, obj.Ts);
        end
        
        function S = plus(S1, S2)
            % PLUS Addition
            % Get sampling time of first non-static input
            if isa(S1, 'lpvlfr') && ...
                    ~isstatic(S1)
                Ts = S1.Ts;
            else
                Ts = S2.Ts;
            end
            
            % Transfer input and output properties
            if isa(S1, 'lpvlfr')
                source = S1;
            else
                source = S2;
            end
            
            if ~isa(S1, 'lpvlfr')
                S1 = lpvss(S1);
                S1.Ts = Ts;
            end
            if ~isa(S2, 'lpvlfr')
                S2 = lpvss(S2);
                S2.Ts = Ts;
            end
            
            assert(S1.Ts == S2.Ts)
            [S1, S2] = lpvlfr.commonrho_(S1, S2);
            % construct matrices
            SA = blkdiag(S1.A0, S2.A0);
            SB = [blkdiag(S1.Bw, S2.Bw), [S1.Bu; S2.Bu]];
            SC = [blkdiag(S1.Cz, S2.Cz); [S1.Cy, S2.Cy]];
            SD = [blkdiag(S1.Dzw, S2.Dzw), [S1.Dzu; S2.Dzu]; ...
                [S1.Dyw, S2.Dyw], S1.Dyu + S2.Dyu];
            g = ss(SA, SB, SC, SD, S1.Ts);
            delta = blkdiag(S1.Delta_, S2.Delta_);
            S = S1;
            S.Delta_ = delta;
            S.G_ = g;
            S.SchedulingTimeMap_ = delta.timemap;
            S.InputName = source.InputName;
            S.InputUnit = source.InputUnit;
            S.InputDelay = source.InputDelay;
            S.OutputName = source.OutputName;
            S.OutputUnit = source.OutputUnit;
            S.StateName = [S1.StateName; S2.StateName];
            S.StateUnit = [S1.StateUnit; S2.StateUnit];
        end
        
        function S = minus(S1, S2)
            % MINUS Subtraction
            S = plus(S1, -S2);
        end
        
        function S = mtimes(S1, S2)
            if size(S1, 2) ~= size(S2, 1)
                if isscalar(S1)
                    S1 = S1 * eye(size(S2, 1));
                elseif isscalar(S2)
                    S2 = S2 * eye(size(S1, 2));
                else
                    error(['Attempting to multiply LPVLFR or LPVSS objects ', ...
                        'with incompatible dimensions.']);
                end
            end
            blksys = append(S1, S2);
            nu1 = size(S1, 2);
            nu2 = size(S2, 2);
            ny1 = size(S1, 1);
            ny2 = size(S2, 1);
            % First nu1 inputs connected to last ny2 outputs
            connections = [ (1:nu1)', ...
                (ny1+1:ny1+ny2)' ];
            inputs = (nu1+1):(nu1+nu2);
            outputs = 1:ny1;
            S = connect(blksys, connections, inputs, outputs);
        end
        
        function C = mpower(A, b)
            %MPOWER Matrix power with integer scalar
            %
            %   Syntax:
            %       A^b
            %
            %   Inputs:
            %       A: lpvlfr
            %       b (int): positive integer
            %
            %   Outputs:
            %       C: A^b
            if ~isscalar(b) || ~isnumeric(b) ...
                    || b ~= round(b) || b < 0
                error(['Matrix power can only be used with ', ...
                    'lpvss for non-negative, scalar, integer B.']);
            end
            if b == 0
                CA = zeros(A.Nx, A.Nx); CB = zeros(A.Nx, A.Nu + A.Nw);
                CC = zeros(A.Ny + A.Nz, A.Nx); CD = blkdiag(zeros(A.Nz), eye(A.Ny));
                A.G_ = ss(CA, CB, CC, CD, A.Ts);
                C = A;
                return;
            end
            C = A;
            for i=2:b
                C = mtimes(C, A);
            end
        end
        
        function [y, t, x] = lsim(obj, p, u, varargin)
            % LSIM Time-domain simulation
            %
            % Syntax:
            %   [y, t, x] = lsim(sys, p, u)
            %   [y, t, x] = lsim(sys, p, u, t)
            %

            % Parse options
            lsimOpts = LPVcore.parselsimOpts(obj, p, u, varargin{:});
            if isct(obj)
                if obj.Np == 0
                    error(['Continuous-time simulation is not yet ', ...
                        'supported for systems with Np = 0, but will ', ...
                        'be in a future version.']);
                end
                [y, t, x] = ctsim_(obj, p, u, lsimOpts.t, lsimOpts.x0, ...
                        lsimOpts.Solver, lsimOpts.SolverOpts, lsimOpts.SolverMode);
            else
                [y, t, x] = dtsim_(obj, p, u, lsimOpts.x0);
            end
            if nargout == 0
                figure;
                plotlsim(obj, y, p, u, t);
            end
        end
        
        function S = lft(S1, S2, nu, ny)
            % LFT Generalized feedback interconnection of two models
            % (Redheffer Star Product)
            %
            %   Syntax:
            %       S = lft(S1, S2, nu, ny)
            %       S = lft(S1, S2)
            %
            %   Inputs:
            %       S1, S2: lpvlfr models
            %       nu: first nu outputs of S2 are connected to last nu
            %       inputs of S1.
            %       ny: last ny outputs of S1 are connected to first ny
            %       inputs of S2.
            %
            %   See https://nl.mathworks.com/help/control/ref/lft.html.
            %
            
            % Get sampling time of first non-static input
            if isa(S1, 'lpvlfr') && ...
                    ~isstatic(S1)
                Ts = S1.Ts;
            else
                Ts = S2.Ts;
            end
            % Convert
            if ~isa(S1, 'lpvlfr')
                S1 = lpvss(S1);
                S1.Ts_ = Ts;
            end
            if ~isa(S2, 'lpvlfr')
                S2 = lpvss(S2);
                S2.Ts_ = Ts;
            end
            if nargin == 2
                if S2.Nu < S1.Nu && S2.Ny < S1.Ny
                    % Lower LFT
                    nu = S2.Ny;
                    ny = S2.Nu;
                elseif S1.Nu < S2.Nu && S1.Ny < S2.Ny
                    % Upper LFT
                    nu = S1.Nu;
                    ny = S1.Ny;
                else
                    error('LFT with incompatible nr. of inputs and outputs.');
                end
            end
            if nu > S1.Nu || nu > S2.Ny
                error('nu too large.');
            end
            if ny > S1.Ny || ny > S2.Nu
                error('ny too large.');
            end
            if (~isstatic(S1) && ~isstatic(S2)) && S1.Ts ~= S2.Ts
                error('Sampling time for S1 and S2 differ.');
            end
            [S1, S2] = lpvlfr.commonrho_(S1, S2);
            S = lft_(S1, S2, nu, ny);
        end
        
        function S = append(varargin); S = blkdiag(varargin{:}); end
        
        function S = blkdiag(varargin)
            % BLKDIAG Block-diagonal interconnection of systems
            
            % Get sampling time of first non-static input
            for i=1:nargin
                if isa(varargin{i}, 'lpvlfr') && ...
                        ~isstatic(varargin{i})
                    Ts = varargin{i}.Ts;
                    break;
                end
            end
            
            for i=1:nargin
                if ~isa(varargin{i}, 'lpvlfr')
                    varargin{i} = lpvss(varargin{i});
                    varargin{i}.Ts_ = Ts;
                end
            end
            
            % Check sampling time matches
            ts = NaN(1, nargin);
            for i=1:nargin
                if ~isstatic(varargin{i})
                    ts(i) = varargin{i}.Ts;
                end
            end
            if ~all(diff(ts(~isnan(ts))) == 0)
                error('Attempting to append systems with different sampling time.');
            end
            
            % Iteratively concatenate pairs of systems
            S = varargin{1};
            for i=2:nargin
                S = blkdiag_(S, varargin{i});
            end
            
        end
        
        function v = isstatic(obj)
            % ISSTATIC Returns whether LPV-LFR system represents static
            % gain
            v = obj.Nx == 0;
        end
        
        % abstract functions from lpvrep
        function val = getNy(obj); val = size(obj.G_.C,1)-obj.Nz; end
        function val = getNu(obj); val = size(obj.G_.B,2)-obj.Nw; end
        
        % set functions
        function obj = set.G(obj,newG)
            % check number of inputs and outputs
            [outDim,inDim] = size(newG);
            if ~all([obj.Nu+obj.Nw,obj.Ny+obj.Nz] == [inDim,outDim])
                error(['Cannot set G, number of input must be ', int2str(obj.Nu), ', number of outputs must be ', int2str(obj.Ny)]);
            end
            
            % check if sample time equals the original G
            if obj.G_.Ts ~= newG.Ts
                warning('Update of sys.G changes sampling time');
            end
            obj.G_ = newG;
        end
        
        function obj = set.Delta(obj,newDelta)
            deltaRow = size(newDelta.matrices,1);
            deltaCol = size(newDelta.matrices,2);
            % size check
            if ~all([obj.Nw, obj.Nz] == [deltaRow, deltaCol])
                error(['Nw must be of size ', int2str(obj.Nw), ', Nz must be of size ', int2str(obj.Nz)]);
            end
            % sample time check
            if ~strcmp(obj.Delta.timemap.domain,newDelta.timemap.domain)
                warning('The Delta that is being set uses a different time domain (ct/dt)')
            end
            obj.Delta_ = newDelta;
            obj.SchedulingTimeMap_ = newDelta.timemap;
        end
        
        function obj = set.StateName(obj, value)
            obj.StateName_ = LPVcore.cellarrayProperty('StateName', value, obj.Nx);
        end
        
        function obj = set.StateUnit(obj, value)
            obj.StateUnit_ = LPVcore.cellarrayProperty('StateUnit', value, obj.Nx);
        end
        
        % get functions
        function val = get.G(obj); val = obj.G_; end
        function val = get.Delta(obj); val = obj.Delta_; end
        function val = get.Nx(obj); val = size(obj.G_.A,1); end
        function val = get.Nw(obj); val = size(obj.Delta, 1); end
        function val = get.Nz(obj); val = size(obj.Delta, 2); end
        function val = get.A0(obj); val = obj.G_.A; end
        function val = get.Bw(obj); val = obj.G_.B(:,1:obj.Nw); end
        function val = get.Bu(obj); val = obj.G_.B(:,(obj.Nw+1):(obj.Nw+obj.Nu)); end
        function val = get.Cz(obj); val = obj.G_.C(1:obj.Nz,:); end
        function val = get.Cy(obj); val = obj.G_.C((obj.Nz+1):(obj.Nz+obj.Ny),:); end
        function val = get.Dzw(obj); val = obj.G_.D(1:obj.Nz,1:obj.Nw); end
        function val = get.Dzu(obj); val = obj.G_.D(1:obj.Nz,(1+obj.Nw):(obj.Nw+obj.Nu)); end
        function val = get.Dyw(obj); val = obj.G_.D((1+obj.Nz):(obj.Nz+obj.Ny),1:obj.Nw); end
        function val = get.Dyu(obj); val = obj.G_.D((1+obj.Nz):(obj.Nz+obj.Ny),(1+obj.Nw):(obj.Nw+obj.Nu)); end
        function val = get.StateName(obj); val = obj.StateName_; end
        function val = get.StateUnit(obj); val = obj.StateUnit_; end
        sysd = c2d(sysc, Ts, method, opts);
    end
    
    methods(Hidden, Access='protected')
        [y, t, x] = ctsim_(obj, p, u, tin, x0, solver, solverOpts, solverMode)
        [y, t, x] = dtsim_(obj, p, u, x0)
        
        function sys = extractLocal_(obj, p)
            % EXTRACTLOCAL_ See lpvrep. Returns ss.
            sys = lft(freeze(obj.Delta_, p), obj.G_);
            % Transfer properties
            properties = {'InputName', 'OutputName', ...
                'InputUnit', 'OutputUnit', 'InputDelay', ...
                'StateName', 'StateUnit', 'Name', 'Notes', ...
                'TimeUnit'};
            for i=1:numel(properties)
                sys.(properties{i}) = obj.(properties{i});
            end
        end
        
        function S = cat_(dim, S1, S2)
            %CAT_ Concatenates 2 lpvlfr objects with common ext.
            %scheduling variable.
            %
            %   Synatx:
            %       S = cat_(dim, A, B)
            %
            %   Inputs:
            %       dim (int): dimension along which to concatenate
            %       A, B: lpvlfr objects to concatenate. Assumptions:
            %           * same ext. scheduling variable (commonrho)
            %
            %   Outputs:
            %       C: concatenated lpvlfr object
            %
            if isempty(S1)
                S = S2;
                return;
            end
            if isempty(S2)
                S = S1;
                return;
            end
            if dim == 1 % vertical
                SA = blkdiag(S1.A0, S2.A0);
                SB = [blkdiag(S1.Bw, S2.Bw), [S1.Bu; S2.Bu]];
                SC = [blkdiag(S1.Cz,S2.Cz); blkdiag(S1.Cy, S2.Cy)];
                SD = [blkdiag(S1.Dzw, S2.Dzw), [S1.Dzu; S2.Dzu]; blkdiag(S1.Dyw,S2.Dyw), [S1.Dyu; S2.Dyu]];
                sysVertcat = ss(SA,SB,SC,SD,S1.Ts);
                deltaVertcat = blkdiag(S1.Delta_, S2.Delta_);
                S = lpvlfr(deltaVertcat,sysVertcat);
                % The properties that should be concatenated
                catProperties = {'OutputName', 'OutputUnit'};
                % The properties that should be the same (otherwise, a
                % warning is thrown)
                sameProperties = {'InputName', 'InputUnit', 'InputDelay'};
            elseif dim == 2 % horizontal
                SA = blkdiag(S1.A0, S2.A0);
                SB = [blkdiag(S1.Bw, S2.Bw), blkdiag(S1.Bu, S2.Bu)];
                SC = [blkdiag(S1.Cz, S2.Cz); [S1.Cy, S2.Cy]];
                SD = [blkdiag(S1.Dzw, S2.Dzw), blkdiag(S1.Dzu, S2.Dzu); [S1.Dyw, S2.Dyw], [S1.Dyu, S2.Dyu]];
                sysHorcat = ss(SA, SB, SC, SD, S1.Ts);
                deltaHorcat = blkdiag(S1.Delta, S2.Delta);
                S = lpvlfr(deltaHorcat,sysHorcat);
                % The properties that should be concatenated
                catProperties = {'InputName', 'InputUnit', 'InputDelay'};
                % The properties that should be the same (otherwise, a
                % warning is thrown)
                sameProperties = {'OutputName', 'OutputUnit'};
            else
                error('Trying to concatenate lpvlfr objects with invalid dim.');
            end
            % Transfer properties
            catProperties = [catProperties(:)', {'StateName'}, {'StateUnit'}];
            for i=1:numel(catProperties)
                S.(catProperties{i}) = [S1.(catProperties{i}); S2.(catProperties{i})];
            end
            for i=1:numel(sameProperties)
                if ~isequal(S1.(sameProperties{i}), S2.(sameProperties{i}))
                    warning(['Ignoring ', sameProperties{i}, ' because of name conflicts.']);
                    S.(sameProperties{i}) = {};
                else
                    S.(sameProperties{i}) = S1.(sameProperties{i});
                end
            end
        end
        
        function S = blkdiag_(S1, S2)
            delta = blkdiag(S1.Delta, S2.Delta);
            
            SA = blkdiag(S1.A0, S2.A0);
            SB = [blkdiag(S1.Bw, S2.Bw), blkdiag(S1.Bu, S2.Bu)];
            SC = [blkdiag(S1.Cz, S2.Cz); blkdiag(S1.Cy, S2.Cy)];
            SD = [blkdiag(S1.Dzw, S2.Dzw), blkdiag(S1.Dzu, S2.Dzu); ...
                blkdiag(S1.Dyw, S2.Dyw), blkdiag(S1.Dyu, S2.Dyu)];
            g = ss(SA, SB, SC, SD, S1.Ts);
            
            if isa(S1, 'lpvlfr')
                S = S1;
            else
                S = S2;
            end
            S.Delta_ = delta;
            S.G_ = g;
            if ~isempty(delta)
                S.SchedulingTimeMap_ = delta.timemap;
            else
                if g.Ts == 0
                    S.SchedulingTimeMap_ = timemap([], 'ct');
                else
                    S.SchedulingTimeMap_ = timemap([], 'dt');
                end
            end
            
            % Transfer stackable properties
            properties = {'InputName', 'OutputName', ...
                'InputUnit', 'OutputUnit', 'InputDelay', ...
                'StateName', 'StateUnit'};
            for i=1:numel(properties)
                property = cellfun(@(x) x.(properties{i}), {S1, S2}, 'UniformOutput', false);
                S.(properties{i}) = vertcat(property{:});
            end
        end
        
        function sys = connect_(sys, connections)
            % CONNECT_ Index-based connection
            %
            %   Syntax:
            %       S = connect_(S, connections)
            %
            %   Inputs:
            %       connections: matrix specifying connections. For each
            %       row, the first column indicates the input port that
            %       gets connected to the output port specified by the
            %       remainder of the row (with a sign). E.g. [7, 2, -15, 6]
            %       specifies that y(2) - y(15) + y(6) feeds into u(7).
            %
            N = size(connections, 1);
            S = zeros(sys.Nu, sys.Ny);
            for i=1:N
                in = connections(i, 1);
                for j=2:size(connections, 2)
                    if connections(i, j) ~= 0 % 0 is not connected
                        out = connections(i, j);
                        S(in, abs(out)) = sign(out);
                    end
                end
            end
            % Check for singular loops in connection
            if rank(eye(sys.Ny) - sys.Dyu * S) < sys.Ny
                warning('The closed-loop model may be improper because of singular algebraic loops.');
            end
            sys = connectmat_(sys, S);
        end
        
        function obj = connectmat_(obj, S)
            % CONNECTMAT_ Index-based connection with matrix
            %   Utility function for connecting output ports to input ports
            %
            %                    +--------+
            %                    |        |
            %             +------+ Delta  +<-------+
            %             |      |        |        |
            %             |      +--------+        |
            %           w |                        | z
            %             |   +---------------+    |
            %             |   |               |    |
            %      +---+  +-->+               +----+
            % ubar |   |      |               |        y
            %  --->+ + +----->+      G        +----+----->
            %      |   |   u  |               |    |
            %      +-+-+      |               |    |
            %        ^        +---------------+    |
            %        |                             |
            %        |          +-----------+      |
            %        |  Sy      |           |   y  |
            %        |          |           |      |
            %        +----------+     S     +<-----+
            %                   |           |
            %                   |           |
            %                   +-----------+
            %
            %   Syntax:
            %       sysc = connectsingle_(sys, S)
            %
            %   Inputs:
            %       sys: lpvlfr system to connect
            %       S: matrix u = S y to add to input channel
            %
            
            Stilde = S / (eye(obj.Ny) - obj.Dyu * S);
            a0 = obj.A0 + obj.Bu * Stilde * obj.Cy;
            bw = obj.Bw + obj.Bu * Stilde * obj.Dyw;
            bu = obj.Bu + obj.Bu * Stilde * obj.Dyu;
            cz = obj.Cz + obj.Dzu * Stilde * obj.Cy;
            dzw = obj.Dzw + obj.Dzu * Stilde * obj.Dyw;
            dzu = obj.Dzu + obj.Dzu * Stilde * obj.Dyu;
            cy = (eye(obj.Ny) - obj.Dyu * S) \ obj.Cy;
            dyw = (eye(obj.Ny) - obj.Dyu * S) \ obj.Dyw;
            dyu = (eye(obj.Ny) - obj.Dyu * S) \ obj.Dyu;
            obj.G_.A = a0;
            obj.G_.B = [bw, bu];
            obj.G_.C = [cz; cy];
            obj.G_.D = [dzw, dzu; ...
                dyw, dyu];
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

        function sys = addFeedthrough_(sys, inputName, idx)
            % Add direct feedthrough from input specified by inputName to
            % output at index idx. Increase total number of outputs of sys
            % by 1.
            if ~any(strcmp(inputName, sys.InputName))
                error(['Input ''', inputName, ''' does not exist']);
            end
            assert(isint(idx) && idx > 0, '''idx'' must be an index (i.e., positive integer)');
            if idx > (size(sys, 1) + 1)
                error(['Index ''', int2str(idx), ''' is too big for output size']);
            end
            % Find index of input
            idx_input = find(strcmp(inputName, sys.InputName));
            % Convert indices of input and output to indices of
            % LFR-representation (i.e., incl. w and z signals)
            idx_output_lfr = idx + sys.Nz;
            idx_input_lfr = idx_input + sys.Nw;
            % Insert output at idx
            Cnew = [sys.G.C(1:idx_output_lfr-1, :); zeros(1, size(sys.G.C, 2)); ...
                sys.G.C(idx_output_lfr:end, :)];
            Dnew = [sys.G.D(1:idx_output_lfr-1, :); zeros(1, size(sys.G.D, 2)); ...
                sys.G.D(idx_output_lfr:end, :)];
            Dnew(idx_output_lfr, idx_input_lfr) = 1;
            sys.G_ = ss(sys.G.A, sys.G.B, Cnew, Dnew, sys.G.Ts);
            % Update signal properties
            sys.OutputName_ = ...
                [sys.OutputName(1:idx-1); {inputName}; sys.OutputName(idx:end)];
            sys.OutputUnit_ = ...
                [sys.OutputUnit(1:idx-1); sys.InputUnit(idx_input); sys.OutputUnit(idx:end)];
        end
    end
end

