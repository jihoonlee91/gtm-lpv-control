classdef (Abstract) lpvrep
%LPVREP Base class for LPV system representations such as LPVIO, LPVSS, LPVIDPOLY and LPVIDSS.
%
%   This class contains all common properties and associated get/set
%   functions of the LPVIO, LPVSS, LPVIDPOLY, LPVIDSS class definitions
%
%    See also LPVIO, LPVSS, LPVIDPOLY, LPVIDSS.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).    
    
    properties (Dependent)
        
        % Input delay for each input channel, specified as a scalar value
        % or numeric vector. NOTE: currently, only a scalar value is allowed (i.e., the same input delay for each channel.) 
        % For continuous-time systems, specify input
        % delays in the time unit stored in the TimeUnit property. For
        % discrete-time systems, specify input delays in integer multiples
        % of the sample time Ts. For example, InputDelay = 3 means a delay
        % of three sample times.
        %
        % For a system with Nu inputs, set InputDelay to an Nu-by-1 vector.
        % Each entry of this vector is a numerical value that represents
        % the input delay for the corresponding input channel.
        %
        % You can also set InputDelay to a scalar value to apply the same
        % delay to all channels.
        %
        % Default: 0
        InputDelay
        
        % Sample time. For continuous-time models, Ts = 0. For
        % discrete-time models, Ts is a positive scalar representing the
        % sampling period. This value is expressed in the unit specified by
        % the TimeUnit property of the model. To denote a discrete-time
        % model with unspecified sample time, set Ts = -1.
        % 
        % Changing this property does not discretize or resample the model.
        % 
        % Default: -1 (discrete time)
        Ts
                 	
        % Units for the time variable, the sample time Ts, and any time
        % delays in the model, specified as one of the following values:
        % 
        % 'nanoseconds', 'microseconds', 'milliseconds', 'seconds',
        % 'minutes', 'hours', 'days', 'weeks', 'months', 'years'
        % 
        % Changing this property has no effect on other properties, and
        % therefore changes the overall system behavior. Use chgTimeUnit to
        % convert between time units without modifying system behavior.
        %
        % Default: 'seconds'
        TimeUnit
        
        % Input channel names, specified as one of the following:
        % 
        % - Character vector. For single-input models, for example,
        %   'controls'.
        % - Cell array of character vectors. For multi-input models.
        % 
        % Alternatively, use automatic vector expansion to assign input
        % names for multi-input models. For example, if sys is a two-input
        % model, enter:
        %   sys.InputName = 'controls';
        %
        % The input names automatically expand to
        % {'controls(1)';'controls(2)'}.
        % 
        % You can use the shorthand notation u to refer to the InputName
        % property. For example, sys.u is equivalent to sys.InputName. 
        % 
        % Input channel names have several uses, including:
        % - Identifying channels on model display and plots
        % - Extracting subsystems of MIMO systems
        % 
        % Specifying connection points when interconnecting models
        % 
        % Default: '' for all input channels
        InputName
        
        % Input channel units, specified as one of the following:
        % - Character vector. For single-input models, for example,
        %   'seconds'. 
        % - Cell array of character vectors. For multi-input models.
        % 
        % Use InputUnit to keep track of input signal units. InputUnit has
        % no effect on system behavior.
        % 
        % Default: '' for all input channels
        InputUnit
        
        	
        % Input channel groups. The InputGroup property lets you assign the
        % input channels of MIMO systems into groups and refer to each
        % group by name. Specify input groups as a structure. In this
        % structure, field names are the group names, and field values are
        % the input channels belonging to each group. For example: 
        %    sys.InputGroup.controls = [1 2];
        %    sys.InputGroup.noise = [3 5];
        %
        % creates input groups named controls and noise that include input
        % channels 1, 2 and 3, 5, respectively. You can then extract the
        % subsystem from the controls inputs to all outputs using:
        % 
        % sys(:,'controls')
        % Default: Struct with no fields
        InputGroup
        
        % Output channel names, specified as one of the following:
        % - Character vector. For single-output models. For example,
        %   'measurements'.
        % - Cell array of character vectors. For multi-output models.
        % 
        % Alternatively, use automatic vector expansion to assign output
        % names for multi-output models. For example, if sys is a
        % two-output model, enter:
        %    sys.OutputName = 'measurements';
        %
        % The output names automatically expand to {'measurements(1)';
        % 'measurements(2)'}.
        % 
        % You can use the shorthand notation y to refer to the OutputName
        % property. For example, sys.y is equivalent to sys.OutputName.
        % 
        % Output channel names have several uses, including:
        % - Identifying channels on model display and plots
        % - Extracting subsystems of MIMO systems
        % 
        % Default: '' for all output channels
        OutputName
        
        % Output channel units, specified as one of the following:
        % - Character vector. For single-output models. For example,
        %   'seconds'.
        % - Cell array of character vectors. For multi-output models.
        % 
        % Use OutputUnit to keep track of output signal units. OutputUnit has no effect on system behavior.
        % 
        % Default: '' for all output channels
        OutputUnit
                	
        % Output channel groups. The OutputGroup property lets you assign
        % the output channels of MIMO systems into groups and refer to each
        % group by name. Specify output groups as a structure. In this
        % structure, field names are the group names, and field values are
        % the output channels belonging to each group. For example:
        %    sys.OutputGroup.temperature = [1];
        %    sys.OutputGroup.measurement = [3 5];
        %
        % creates output groups named temperature and measurement that
        % include output channels 1, and 3, 5, respectively. You can then
        % extract the subsystem from all inputs to the measurement outputs
        % using:
        %    sys('measurement',:)
        %
        % Default: Struct with no fields
        OutputGroup
        
        % System name, specified as a character vector. For example,
        % 'system_1'.
        % 
        % Default: ''
        Name
                	
        % Any text that you want to associate with the system, stored as a
        % string or a cell array of character vectors. The property stores
        % whichever data type you provide. For instance, if sys1 and sys2
        % are dynamic system models, you can set their Notes properties as
        % follows:
        %   sys1.Notes = "sys1 has a string.";
        %   sys2.Notes = 'sys2 has a character vector.';
        %   sys1.Notes
        %   sys2.Notes
        %
        % ans = 
        % 
        %     "sys1 has a string."
        % 
        % ans =
        % 
        %     'sys2 has a character vector.'
        % 
        % Default: [0?1 string]
        Notes   
        
        % Scheduling time-map
        SchedulingTimeMap
        
        % Names of scheduling signals
        SchedulingName
        
        % Units of scheduling signals
        SchedulingUnit

    end  % end of public, dependent properties
    
    properties (Dependent, SetAccess = protected)
        % Dimension of output signal
        Ny
        
        % Dimension of scheduling signal
        Np
        
        % Dimension of extended scheduling signal
        Nrho
        
        % Dimension of input signal
        Nu
           
        % Dimension of scheduling signals
        SchedulingDimension
    end
    
    properties (Access = public)
        % Any type of data you want to associate with system, specified as
        % any MATLAB data type.
        % 
        % Default: []
        UserData = [];   % Litterally, we don't verify its value
    end

    properties (Access = private, Constant = true)
        
        % Set of available time unites for TimeUnit, InputUnit, OutputUnit
        unitsTime = {'nanoseconds', 'microseconds', 'milliseconds', ...
            'seconds', 'minutes', 'hours', 'days', 'weeks', 'months', ...
            'years'};
    end
    
	properties (Hidden, Dependent)
      % Input names. Alias for "InputName" property.
      u
      % Output names. Alias for "OutputName" property.
      y
    end
   
	properties (Access = protected, Hidden)
            % n-by-1 matrix with unsigned integers
        InputDelay_ = [];
            % Positive real number or -1
        Ts_ = -1;
            % String
        TimeUnit_ = 'seconds';
            % Cell array of strings
        InputName_ = {};
            % Cell array of strings
        InputUnit_ = {};
            % Structure
        InputGroup_ = struct();
            % Cell array of strings
        OutputName_ = {};
            % Cell array of strings
        OutputUnit_ = {};
            % Structure
        OutputGroup_ = struct();
            % String
        Name_ = '';
            % String or Cell array of strings
        Notes_ = strings([0 1]);
            % Scheduling timemap
        SchedulingTimeMap_ = timemap;
   end
    
    methods
        function obj = lpvrep()
        end
        
        function sys = extractLocal(obj, p)
            % EXTRACTLOCAL Get frozen models at operating points
            %
            %   Syntax:
            %       sys = extractLocal(lpvrep, p)
            %       {sys1, ..., sysn} = extractLocal(lpvrep, {p1, ..., pn})
            %       sysarray = extractLocal(lpvrep, s)
            %
            %   Inputs:
            %       p, p1, ..., pn: 1-by-np vector of frozen scheduling values
            %       s: structure specifying a rectangular grid of operating
            %           points. Each field in the structure corresponds
            %           to a scheduling signal. Example:
            %               s = struct('p', [0, 1], 'q', [-1, 0])
            %           evaluates the LPV representation at 4 operating
            %           points (each combination of p = 0 / 1 and q = -1 /
            %           0). When this input is used, the return value is a
            %           state-space array.
            %
            %   Outputs:
            %       sys, sys1, ..., sysn: LTI system in representation
            %           specified by child class.
            %       sysarray: model array.
            %

            narginchk(2, 2);
            nargoutchk(0, 1);

            % Distinguish between 2 input modes: regular grid specified as
            % struct, or irregular grid specified as cell array.
            if isstruct(p)
                % Struct mode
                % Check structure and sort fields by scheduling time map
                p = LPVcore.schedulingStructCheck(obj, p);
                % Convert structure to cell grid
                % Grid dimension (nr. of scheduling signals)
                f = fieldnames(p);
                indx = cell(1, obj.Nrho);
                % Size of rectangular grid
                sz = NaN(1, numel(f));
                for i=1:numel(f)
                    sz(i) = numel(p.(f{i}));
                end
                % Number of points
                N = prod(sz);
                % Pre-allocate model array
                if isct(obj)
                    sys(:, :, N) = rss(1, obj.Ny, obj.Nu);
                else
                    sys(:, :, N) = drss(1, obj.Ny, obj.Nu);
                end
                % Iterate over points
                for i=1:N
                    [indx{:}] = ind2sub(sz, i);
                    pi = NaN(1, obj.Nrho);
                    for j=1:obj.Nrho
                        pi(j) = p.(f{j})(indx{j});
                    end
                    sys(:, :, i) = extractLocal_(obj, pi);
                end
                % Reshape model array if needed according to dimension
                if numel(sz) > 1
                    sys = reshape(sys, sz);
                end
            else
                % Cell mode
                cellIn = true;
                if ~iscell(p)
                    cellIn = false;
                    p = {p};
                end
                np = obj.Np;
                sys = cell(1, numel(p));
                for i=1:numel(p)
                    if ~all(size(p{i}) == [1, np]) ...
                            && ~(isempty(p{i}) && np == 0)
                        error('Each frozen scheduling variable must be 1-by-Np');
                    end
                    sys{i} = extractLocal_(obj, p{i});
                    sys{i}.Ts = obj.Ts;
                    sys{i}.InputName = obj.InputName;
                    sys{i}.OutputName = obj.OutputName;
                    sys{i}.InputUnit = obj.InputUnit;
                    sys{i}.OutputUnit = obj.OutputUnit;
                    sys{i}.InputDelay = obj.InputDelay;
                    sys{i}.TimeUnit = obj.TimeUnit;
                end
                % If input isn't cell, output shouldn't be cell
                if ~cellIn
                    sys = sys{1};
                end
            end
        end
        
        function bode(varargin)
            % BODE Plot Bode diagram of LPV system for frozen values of p
            %
            %   Syntax:
            %       bode(sys, p)
            %       bode(sys1, ..., sysm, p)
            %       bode(sys1, LineSpec1, ..., sysm, LineSpecm, p)
            %       bode(___, w)
            %
            %   Inputs:
            %       sys, sys1, ..., sysm: LPV-LFR models
            %       p: N-by-Np matrix. Each row corresponds to a frozen
            %           scheduling signal at which frozen system Bode diagram
            %           should be plotted.
            %       LineSpec1, ..., LineSpecm: line specification for
            %           drawing the Bode diagram.
            %       w: vector of frequencies at which to compute and plot the
            %           frequency response.
            %

            % p and atleast one sys have to be defined.
            narginchk(2, Inf); 
            
            % Parse the input
            bodeArg = parsePlotting(varargin);
            
            % create bode plot
            bode(bodeArg{:});
        end
        
        function bodemag(varargin)
            % BODEMAGNITUDE Plot Bode magnitude diagram of LPV system for frozen values of p
            %
            %   Syntax:
            %       bodemag(sys, p)
            %       bodemag(sys1, ..., sysm, p)
            %       bodemag(sys1, LineSpec1, ..., sysm, LineSpecm, p)
            %       bodemag(___, w)
            %
            %   Inputs:
            %       sys, sys1, ..., sysm: LPV-LFR models
            %       p: N-by-Np matrix. Each row corresponds to a frozen
            %           scheduling signal at which frozen system Bode diagram
            %           should be plotted.
            %       LineSpec1, ..., LineSpecm: line specification for
            %           drawing the Bode diagram.
            %       w: vector of frequencies at which to compute and plot the
            %           frequency response.
            %

            % p and atleast one sys have to be defined.
            narginchk(2, Inf); 
            
            % Parse the input
            bodemagArg = parsePlotting(varargin);
            
            % create bode plot
            bodemag(bodemagArg{:});
        end
        
        function sigma(varargin)
            % SIGMA Plot singular value plot of LPV system for frozen values of p
            %
            %   Syntax:
            %       sigma(sys, p)
            %       sigma(sys1, ..., sysm, p)
            %       sigma(sys1, LineSpec1, ..., sysm, LineSpecm, p)
            %       sigma(___, w)
            %
            %   Inputs:
            %       sys, sys1, ..., sysm: LPV-LFR models
            %       p: N-by-Np matrix. Each row corresponds to a frozen
            %           scheduling signal at which frozen system Bode diagram
            %           should be plotted.
            %       LineSpec1, ..., LineSpecm: line specification for
            %           drawing the Bode diagram.
            %       w: vector of frequencies at which to compute and plot the
            %           frequency response.
            %

            % p and atleast one sys have to be defined.
            narginchk(2, Inf); 
            
            % Parse the input
            sigmaArg = parsePlotting(varargin);
            
            % create bode plot
            sigma(sigmaArg{:});
        end
        
        function [resp, wout] = freqresp(varargin)
            %check if the function has atleast two inputs: a system and a
            %scheduling variable.
            narginchk(2, Inf);
            if ~isa(varargin{1},'lpvrep')
                error('first input should be a lpvrep object')
            end
            % extract scheduling variable and frequencies           
            if (length(varargin)==2) && isa(varargin{end}, 'double')            
                p = varargin{end};
            elseif (length(varargin) == 3) && isa(varargin{end}, 'double') && isa(varargin{end-1},'double')
                w = varargin{end-1};
                p = varargin{end};
            elseif (length(varargin) == 4) && isa(varargin{end}, 'double') && isa(varargin{end-1}, 'char') && isa(varargin{end-2}, 'double')            
                w = varargin{end-2};
                units = varargin{end-1};
                p = varargin{end}; 
            else
                error('invalid syntax, use: freqresp(lvprep obj,p), freqresp(lpvrep obj, w, p) or freqresp(lpvrep obj, w, units, p)');
            end           
            
            % extract number of systems            
            N = size(p, 1);
            
            % loop over all systems and values of p.
            out = cell(N,1);
            sys = varargin{1};
            
            for i=1:N
                sysLocal = extractLocal(sys,p(i, :));
                if exist('w','var') == 1 && exist('units','var')
                    out{i} = freqresp(sysLocal,w,units);
                elseif exist('w','var')
                    out{i} = freqresp(sysLocal,w);
                else
                    [H, wout] = freqresp(sysLocal);
                    out{i} = H;
                end                
            end
            resp = out;
            if exist('w','var')
                wout = w;
            end
        end
        % -----------------------------
        % Get Set operators properties
        
        function val = get.InputDelay(obj)
            val = obj.InputDelay_;
        end
        
        function obj = set.InputDelay(obj,InputDelay)
            
            nu = obj.Nu;
            
            if ~isvector(InputDelay) || any(~isnumeric(InputDelay)) || any(uint16(InputDelay) ~= InputDelay)
                LPVcore.error('LPVcore:general:InvalidProperty','InputDelay', 'nu x 1 vector with integer greater or equal to 0',class(obj));
            end
            
                % Set default value if empty
            if isempty(InputDelay) || nu == 0
                obj.InputDelay_ = zeros(nu, 1); 
                return; 
            end
            
            if isrow(InputDelay); InputDelay = InputDelay'; end
            nu_ = size(InputDelay,1);
            
            % TODO: Test this function with the LPVIO or LPVSS class
            if nu_ == nu
                obj.InputDelay_ = InputDelay;
            elseif nu_ == 1 && nu ~= 0 % if scalar, create vector of scalars
                obj.InputDelay_ = ones(nu,1)*InputDelay;
            else
                LPVcore.error('LPVcore:general:InvalidProperty','InputDelay',[sprintf('%d',nu),' x 1 vector with integer greater or equal to 0'],class(obj));
            end
        end
        
        function tf = isct(obj)
            % ISCT Return whether system is continuous-time
            tf = obj.Ts_ == 0;
        end
        
        function tf = isdt(obj)
            % ISDT Return whether system is discrete-time
            tf = ~isct(obj);
        end
        
        function tf = issiso(obj)
            tf = size(obj, 1) == 1 && size(obj, 2) == 1;
        end
        
        function tf = hasStaticSchedulingDependence(obj)
            % HASSTATICSCHEDULINGDEPENDENCDE Returns whether the system
            % representation has static dependence on the scheduling
            % variable
            tf = all(obj.SchedulingTimeMap.Map == 0);
        end
        
        function val = get.Ts(obj)
            val = obj.Ts_;
        end
        
        function obj = set.Ts(obj,Ts)
            % Set the sampling time
            %
            % Small workaround to be able to overload this function by the
            % subclass.
            
            try obj = obj.set_Ts_value(Ts);
            catch E; throw(E);              end
        end
          
        function val = get.TimeUnit(obj)
            val = obj.TimeUnit_;
        end
        
        function obj = set.TimeUnit(obj,TimeUnit)

            if ~isempty(TimeUnit) && (~isrow(TimeUnit) || ~ischar(TimeUnit))
                LPVcore.error('LPVcore:general:InvalidProperty','TimeUnit', ['charachter string of either of the following: ',strjoin(obj.unitsTime,', ')],class(obj));
            end
            
            if any(strcmp(TimeUnit,obj.unitsTime))
                obj.TimeUnit_ = TimeUnit;
            elseif isempty(TimeUnit)
                obj.TimeUnit_ = 'seconds';
            else
                LPVcore.error('LPVcore:general:InvalidProperty','TimeUnit', ['charachter string of either of the following: ',strjoin(obj.unitsTime,', ')],class(obj));
            end
        end
        
        function val = get.InputName(obj)
            val = obj.InputName_;
        end
        
        function obj = set.InputName(obj,InputName)
            obj.InputName_ = LPVcore.cellarrayProperty('InputName', InputName, obj.Nu);
        end
             
        function val = get.InputUnit(obj)
            val = obj.InputUnit_;
        end
        
        function obj = set.InputUnit(obj,InputUnit)
            obj.InputUnit_ = LPVcore.cellarrayProperty('InputUnit', InputUnit, obj.Nu);
        end
        
        function val = get.InputGroup(obj)
            val = obj.InputGroup_;
        end
        
        function obj = set.InputGroup(obj,InputGroup)
            
            nu = obj.Nu;
            
            if isempty(InputGroup)
                obj.InputGroup_ = struct(); return;
            elseif isstruct(InputGroup)
                % Remove empty groups
                InputGroup = localRemoveEmptyGroups(InputGroup);
            else
                LPVcore.error('LPVcore:general:InvalidProperty','InputGroup', 'structure',class(obj));
            end 
 
            if nu == 0; obj.InputGroup_ = struct(); return; end  % Validate input a bit
            
            f = fieldnames(InputGroup);
            
            for ct=1:length(f)
                channels = InputGroup.(f{ct});
                if ~isnumeric(channels) || ~isvector(channels) || ...
                        isempty(channels) || size(channels,1)~=1 || ...
                        ~isequal(channels,round(channels))
                    LPVcore.error('LPVcore:general:InvalidProperty','InputGroup', 'structure', class(obj));
                elseif any(channels<1) || any(channels>nu)
                    LPVcore.error('LPVcore:general:InvalidInput','Some input channel index is out of range.', class(obj));
                elseif length(unique(channels))<length(channels)
                    LPVcore.error('LPVcore:general:InvalidInput','Some input channel indeces are double defined.',class(obj));
                end
            end
            
            obj.InputGroup_ = InputGroup;
        end
        
        function val = get.OutputName(obj)
            val = obj.OutputName_;
        end
        
        function obj = set.OutputName(obj,OutputName)
            obj.OutputName_ = LPVcore.cellarrayProperty('OutputName', OutputName, obj.Ny);
        end
        
        function val = get.OutputUnit(obj)
            val = obj.OutputUnit_;
        end
        
        function obj = set.OutputUnit(obj,OutputUnit)
            obj.OutputUnit_ = LPVcore.cellarrayProperty('OutputUnit', OutputUnit, obj.Ny);
        end
        
        function val = get.OutputGroup(obj)
            val = obj.OutputGroup_;
        end
        
        function obj = set.OutputGroup(obj,OutputGroup)
            
            ny = obj.Ny;
            
            if isempty(OutputGroup)
                obj.OutputGroup = struct; return;
            elseif isstruct(OutputGroup)
                % Remove empty groups
                OutputGroup = localRemoveEmptyGroups(OutputGroup);
            else
                LPVcore.error('LPVcore:general:InvalidProperty','InputGroup', 'structure', class(obj));
            end
            
            if ny == 0; obj.OutputGroup_ = struct(); return; end  % Validate input a bit
 
            f = fieldnames(OutputGroup);
            
            for ct=1:length(f)
                channels = OutputGroup.(f{ct});
                if ~isnumeric(channels) || ~isvector(channels) || ...
                        isempty(channels) || size(channels,1)~=1 || ...
                        ~isequal(channels,round(channels))
                    LPVcore.error('LPVcore:general:InvalidProperty','InputGroup', 'structure', class(obj));
                elseif any(channels<1) || any(channels>ny)
                    LPVcore.error('LPVcore:general:InvalidInput','Some input channel index is out of range.', class(obj));
                elseif length(unique(channels))<length(channels)
                    LPVcore.error('LPVcore:general:InvalidInput','Some input channel indeces are double defined.', class(obj));
                end
            end
            
            obj.OutputGroup_ = OutputGroup;
        end

        function val = get.Name(obj)
            val = obj.Name_;
        end
        
        function obj = set.Name(obj,Name)
            if isempty(Name)
                obj.Name_ = '';
            elseif (isstring(Name) && isscalar(Name)) || (ischar(Name) && isrow(Name))
                obj.Name_ = Name;
            else
                LPVcore.error('LPVcore:general:InvalidProperty','Name', 'string or 1xn character array',class(obj));
            end
        end
        
        function val = get.SchedulingTimeMap(obj)
            val = obj.SchedulingTimeMap_;
        end
        
        function obj = set.SchedulingTimeMap(obj, val)
            obj.SchedulingTimeMap_ = val;
        end
        
        function val = get.SchedulingDimension(obj)
            val = obj.Np;
        end
        
        function val = get.SchedulingName(obj)
            val = obj.SchedulingTimeMap_.Name;
        end
        
        function obj = set.SchedulingName(obj, val)
            obj.SchedulingTimeMap_.Name = val;
        end
        
        function val = get.SchedulingUnit(obj)
            val = obj.SchedulingTimeMap_.Unit;
        end
        
        function obj = set.SchedulingUnit(obj, val)
            obj.SchedulingTimeMap_.Unit = val;
        end
        
        function val = get.Nu(obj)
            val = getNu(obj);
        end
        
        function val = get.Ny(obj)
            val = getNy(obj);
        end
        
        val = size(obj, varargin);
        
        function val = get.Np(obj)
            val = obj.SchedulingTimeMap_.np;
        end
        
        function val = get.Nrho(obj)
            val = obj.SchedulingTimeMap_.nrho;
        end
        
        function val = get.Notes(obj)
            val = obj.Notes_;
        end
        
        function obj = set.Notes(obj,Notes)
            if isempty(Notes)
                obj.Notes_ = strings([0 1]);
            else
                if ~isstring(Notes)
                    % Accept char or cellstr for backward compatibility
                    try
                        Notes = cellstr(Notes);
                        % NOTE: This function is introduced in 2017b
                        Notes = convertCharsToStrings(Notes);
                    catch
                        LPVcore.error('LPVcore:general:InvalidProperty','Notes', 'string vector, for example, ["a";"b";"c"]',class(obj));
                    end
                end
                obj.Notes_ = Notes(:);
            end
        end
        
        function Value = get.u(obj)
            % u is shorthand for InputName
            Value = obj.InputName;
        end
        
        function Value = get.y(obj)
            % y is shorthand for OutputName
            Value = obj.OutputName;
        end
        
        function obj = set.u(obj,Value)
            try obj.InputName = Value;
            catch E; throw(E); end
        end
        
        function obj = set.y(obj,Value)
            try obj.OutputName = Value;
            catch E; throw(E); end
        end
        
        function varargout = get(obj, varargin)
            %  For documentation, use 'doc InputOutputModel/get'
            
            % TODO: alignment of list
            assert(nargin <= 2 && nargout <= 1, "Please use the supported syntax")
            props = properties(obj);
            
            if nargin == 1
                if nargout == 1
                    for i = 1 : numel(props)
                        varargout{1}.(props{i}) = obj.(props{i});
                    end
                else
                    for i = 1 : numel(props)
                         if isnumeric(obj.(props{i}))
                             disp( [ props{i}, ': ', mat2str(obj.(props{i})) ] );
                         elseif ischar(obj.(props{i}))
                             disp( [ props{i}, ': ', (obj.(props{i})) ] );
                         else
                             disp( [ props{i}, ': [', int2str(size(obj.(props{i}), 1)), ...
                                 'x', ...
                                 int2str(size(obj.(props{i}), 2)), ...
                                 '] ', class(obj.(props{i})) ]);
                         end
                    end
                end
            else
                assert((ischar(varargin{1}) || iscell(varargin{1})) && all(ismember(varargin{1},props)), "'PropertyName' should be the complete name of a property.")
                if iscell(varargin{1})
                    for i = 1 : numel(varargin{1})
                        varargout{1}{i} = obj.(varargin{1}{i});
                    end
                else
                    varargout{1} = obj.(varargin{1});
                end
            end
        end
    end


    methods (Hidden, Access = protected)
        
        function obj = set_Ts_value(obj,Ts)
            % Checking the value of Ts and return it. Small workaround to 
            % be able to overload this function by the subclass.
            if ~isnumeric(Ts) || ( Ts < 0 && Ts ~= -1 )
                LPVcore.error('LPVcore:general:InvalidProperty','Ts','number greater or equal to 0. When the sampling time in unknown, use -1',class(obj));
            end
            % Check compatibility with time-domain
            if isct(obj)
                assert(Ts == 0, 'Ts must be 0 for CT systems');
            end
            if isdt(obj)
                assert(Ts ~=0, 'Ts must not be 0 for DT systems');
            end
            % Update value
            obj.Ts_ = Ts;
        end
        
        function obj = parsePropertyValuePairs(obj, struct)
            % PARSEPROPERTYVALUEPAIRS Set properties from a struct and
            % initialize unset properties
            fn = fieldnames(struct);
            for k=1:numel(fn)
                obj.(fn{k}) = struct.(fn{k});
            end
            obj = setDefaultProperties(obj);
        end
        
    end
    
    methods (Hidden, Access = private)    
        function obj = setDefaultProperties(obj)
            if isempty(obj.InputDelay)
                obj.InputDelay = zeros(obj.Nu, 1);
            end
            if isempty(obj.InputName)
                obj.InputName(1, 1:obj.Nu) = {''};
            end
            if isempty(obj.InputUnit)
                obj.InputUnit(1, 1:obj.Nu) = {''};
            end
            if isempty(obj.OutputName)
                obj.OutputName(1, 1:obj.Ny) = {''};
            end
            if isempty(obj.OutputUnit)
                obj.OutputUnit(1, 1:obj.Ny) = {''};
            end
        end
    end
    
    methods (Hidden, Abstract)
        val = getNu(obj);
        val = getNy(obj);  
    end
    
    methods (Hidden, Abstract, Access = protected)
        sys = extractLocal_(obj, p);
    end

    methods (Hidden, Static)
        function expanded = expandSignalNames_(s, signals)
            % EXPANDSIGNALNAMES_ Expand vector-valued signal names
            %
            % Expands cell-array of input names to explicitly list all its
            % scalar components.
            %
            % Syntax:
            %   s = expandSignalNames_(s, signals)
            %
            % Inputs:
            %   s: cell array of signal names to expand
            %   signals: cell array of all expanded signal names
            %
            % Outputs:
            %   s: expanded version of s
            %
            % Example:
            %       expandSignalNames_({'y'}, {'y(1)', 'y(2)'}) --> {'y(1)', 'y(2)'}
            %
            expanded = {};
            for i=1:numel(s)
                if ~any(strcmp(s{i}, signals))
                    match = regexp(signals, [s{i}, '\(\d+\)']);
                    if all(cellfun(@isempty, match))
                        error('Input ''%s'' does not exist', s{i});
                    else
                        % Vector-valued signal found
                        % Loop over input names and add matched ones
                        for j=1:numel(match)
                            if ~isempty(match{j})
                                expanded = [expanded, signals{j}]; %#ok<AGROW> 
                            end
                        end
                    end
                else
                    expanded = [expanded, s{i}]; %#ok<AGROW> 
                end
            end
        end
    end
    
end



%% -------------------- LOCAL FUNCTIONS -------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function g = localRemoveEmptyGroups(g)
    % Removes groups with empty channel sets
    ie = structfun(@isempty,g);
    if any(ie)
       f = fieldnames(g);
       g = rmfield(g,f(ie));
    end
end

% Convert lpvrep arguments to stack of local LTI models
function argOut = parsePlotting(argIn)
    % check if the first input is a lpvrep object if not exit
    if ~(isa(argIn{1},'lpvrep') || isa(argIn{1},'DynamicSystem'))
        error('first input should be a lpvrep object or LTI system')
    end

    % extract scheduling parameter and initialize input for bode
    if isa(argIn{end}, 'double') && isa(argIn{end-1}, 'double')
        p = argIn{end-1};
        argOut = [argIn(1:end-2),argIn(end)];
    elseif isa(argIn{end}, 'double')
        p = argIn{end};
        argOut = argIn(1:end-1);
    else
        error('Frozen scheduling vector/matrix not given.');
    end   

    % obtain indices of lpvmodel in input argument
    nArg = numel(argIn);

    argSys = zeros(nArg,1);
    for i = 1:nArg
        argSys(i) =  isa(argIn{i},'lpvrep');
    end
    lpvsysInd = find(argSys)';

    % extract number of systems            
    [N,np] = size(p);

    for i = lpvsysInd
        % check if number of scheduling-variables for each system 
        % is correct
        assert(argIn{i}.Np <= np,sprintf('Number of scheduling-variables is incorrect for system %i.',i));

        % get frozen models at grid-points
        sysLocal = extractLocal(argIn{i},mat2cell(p,ones(N,1),np)');
        sysLocalStack = ss([]);
        for j = 1:N
            sysLocalStack(:,:,j) = sysLocal{j};
        end
        argOut{i} = sysLocalStack;
    end
end