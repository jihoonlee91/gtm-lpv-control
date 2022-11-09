classdef lpviddata
%LPVIDDATA Create a data object for input/scheduling/output data and the
%properties of the data set
%
%    data = LPVIDDATA(y,p,u,Ts) to create a data object with output p,
%       scheduling signal p and input U and sample time Ts. Default Ts = 1.
%       y: a N-by-ny matrix where N is the number of observations and ny
%          the number of output channels.
%       p: a N-by-np matrix where np is the number of scheduling channels.
%       u: a N-by-nu matrix, where Nu is the number of input channels.
%       y, p, and u must have the same number of rows. However:
%       If u = [], or not assigned, DAT defines a signal or a time series.
%       If y = [], DAT describes just the input.
%   
%   See also IDDATA.
    
% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).    

    properties (Dependent)
        % Domain - Specifies that the data is in the time domain.
        %
        % Default: 'Time'
        Domain
        
        % ExperimentName - Name of each data set contained in the iddata
        % object.
        %
        % Default: 'Exp1'
        ExperimentName
        
        % InputData - Name of MATLAB variable that stores the input signal
        % to a system.
        %
        % Default: [Matrix of size nu input channels and N data samples]
        InputData
        
        % InputName - Specifies the names of individual input channels.
        % 
        % Default: {'u1';'u2';...}
        InputName
        	
        % InputUnit - Specifies the units of each input channel.
        InputUnit
        
        % SchedulingData - Name of MATLAB variable that stores the
        % scheduling signal to a system.
        %
        % Default: [Matrix of size np scheduling channels and N data samples]
        SchedulingData
        
        % SchedulingName - Specifies the names of individual scheduling
        % channels.
        % 
        % Default: {'p1';'p2';...}
        SchedulingName
        	
        % SchedulingUnit - Specifies the units of each scheduling channel.
        SchedulingUnit
        
        % InterSample - Specifies the behavior of the input signals between
        % samples for transformations between discrete-time and
        % continuous-time.
        % 
        % Currently not used, as the toolbox has no continuous-time
        % identification methods.
        InterSample
        
        % Name - Name of the data set.
        %
        % Default: ''
        Name
        
        % Notes - Comments about the data set.
        % Can be a character vector or cell array.
        %
        % Default: ''
        Notes
        
        % OutputData - Name of MATLAB variable that stores the output
        % signal from a system.
        %
        % Default: [Matrix of size ny output channels and N data samples]
        OutputData
        
        % OutputName - For a multiple-output system, specifies the names of
        % individual output channels.
        %
        % Default: {'y1';'y2';...}
        OutputName
        
        % OutputUnit - 	Specifies the units of each output channel.
        OutputUnit
        
        % Period - Period of the input signal.
        %
        % Default: inf   (nonperiodic signal)
        Period
        
        % TimeUnit - Time unit
        %
        % Default: 'seconds'
        TimeUnit
        
        % Ts - Time interval between successive data samples in seconds.
        %
        % Default: 1
        Ts
        
        % Tstart - Specifies the start time of the time vector.
        %
        % Default: 0
        Tstart
        
        % SamplingInstants - The time values in the time vector calculated
        % from the properties Tstart and Ts.
        SamplingInstants

    end  % end of public, dependent properties
    
    
    properties
        % UserData - Additional comments
        % Can be any MATLAB data type
        % 
        % Default: []
        UserData = [];
    end
    
	properties (Access = private, Constant = true)
        
        % Set of available time unites for TimeUnit, InputUnit, OutputUnit
        unitsTime = {'nanoseconds', 'microseconds', 'milliseconds', ...
            'seconds', 'minutes', 'hours', 'days', 'weeks', 'months', ...
            'years'};
    end
    
	properties (Hidden, Dependent)
      % Alias for "OutputData" property.
      y
      
      % Alias for "SchedulingData" property.
      p
      
      % Alias for "InputData" property.
      u
      
      % Alias for "SamplingInstants" property.
      t
    end
    
    
	properties (Access = protected)
            % String
        Domain_ = 'Time';
            % String
        ExperimentName_ = 'Exp1';
            % N-by-nu matrix
        InputData_ = [];
            % Cell array with strings
        InputName_ = cell(0,1);
            % Cell array with strings
        InputUnit_ = cell(0,1);
            % N-by-np matrix
        SchedulingData_ = [];
            % Cell array with strings
        SchedulingName_ = cell(0,1);
        	% Cell array with strings
        SchedulingUnit_ = cell(0,1);
            % String
        Name_ = '';
            % Character vector or cell array.
        Notes_ = cell(0,1);
            % N-by-ny matrix
        OutputData_ = '';
            % Cell array with string
        OutputName_ = cell(0,1);
            % Cell array with string
        OutputUnit_ = cell(0,1);
            % Positive real number (or inf
        Period_ = inf;
            % String with an element of unitsTime
        TimeUnit_ = 'seconds';
            % Positive number
        Ts_ = 1;
            % Real number
        Tstart_ = 0;
        
        % The length of the data set
        lengthDataSet = 1;
      
        % The ouput, scheduling, and input dimension [ny, np, nu]
        OSISize = [0 0 0];
    end
    
    
    methods
        function obj = lpviddata(y,p,varargin)
            %LPVIDDATA Construct this class
            %   Detailed explanation goes here
            
            % p cannot be empty
            % y can be empty
            % u can be empty.  But u and y empty cannot exist
            
            narginchk(2,30);     % Validate correct number of inputs
            
            n = nargin;
                        
            if n > 4 && mod(n,2) ~= 0
                LPVcore.error('LPVcore:general:InvalidNumOfInputs','lpviddata');
            end
            
            if n >= 5 % Check the PV pairs
                
                listPVpairs = {'ExperimentName', 'InputName', 'InputUnit', 'SchedulingName', 'SchedulingUnit',...
                    'Name', 'Notes', 'OutputName', 'OutputUnit', 'Period', 'TimeUnit', 'Tstart','UserData'};
                tst = LPVcore.validateNamesPVpairs(varargin(3:2:end), listPVpairs);
                
                if tst == false; LPVcore.error('LPVcore:general:NoCharPropertyValuePairs', 'lpviddata');
                elseif ~islogical(tst); LPVcore.error('LPVcore:general:UnknownPropertyValuePairs',varargin{ 2*(tst-1)+3 },'lpviddata'); end
            end
            
                % Validate that at least y or u is nonempty
            if n == 2
                if isempty(y)
                    LPVcore.error('LPVcore:general:InvalidInputArgument','first','matrix of dimension N x ny','lpviddata');
                end
            else
                if isempty(y) && isempty(varargin{1})
                    LPVcore.error('LPVcore:general:InvalidInput','No data input nor output data assigned.','lpviddata');
                end
            end
            
            
            % Validate length of the input/scheduling/output data sets
            N_y = size(y,1);
            N_p = size(p,1);
            if n > 2; N_u = size(varargin{1},1); else; N_u = 0; end
            
            if (N_y > 0 && N_p > 0 && N_y ~= N_p) || (N_u > 0 && N_p > 0 && N_u ~= N_p)
                LPVcore.error('LPVcore:general:InvalidInput','The provided input/scheduling/output data have different lenght.','lpviddata');
            end
            
            obj.lengthDataSet = max([N_y, N_u, N_p]);
            if n == 2; obj.OSISize = [size(y,2), size(p,2), 0]; else; obj.OSISize = [size(y,2), size(p,2), size(varargin{1},2)]; end
            obj.OutputData = y;
            obj.SchedulingData = p;
            if n == 2; obj.InputData = []; else; obj.InputData = varargin{1}; end
            if n >= 4; obj.Ts = varargin{2}; end

            obj.ExperimentName = LPVcore.findProp('ExperimentName', varargin, 3, 'Exp1'); 
            obj.InputName      = LPVcore.findProp('InputName',      varargin, 3, cell(0,1));
            obj.InputUnit      = LPVcore.findProp('InputUnit',      varargin, 3, cell(0,1)); 
            obj.SchedulingName = LPVcore.findProp('SchedulingName', varargin, 3, cell(0,1)); 
            obj.SchedulingUnit = LPVcore.findProp('SchedulingUnit', varargin, 3, cell(0,1)); 
            obj.Name           = LPVcore.findProp('Name',           varargin, 3, ''); 
            obj.Notes          = LPVcore.findProp('Notes',          varargin, 3, cell(0,1)); 
            obj.OutputName     = LPVcore.findProp('OutputName',     varargin, 3, cell(0,1)); 
            obj.OutputUnit     = LPVcore.findProp('OutputUnit',     varargin, 3, cell(0,1)); 
            obj.Period         = LPVcore.findProp('Period',         varargin, 3, inf);
            obj.TimeUnit       = LPVcore.findProp('TimeUnit',       varargin, 3, 'seconds');
            obj.Tstart         = LPVcore.findProp('Tstart',         varargin, 3, 0); 
            obj.UserData       = LPVcore.findProp('UserData',       varargin, 3, []); 
        end
        
        % -----------------------------
        % Get Set operators properties
                
        function val = get.Domain(obj)
            val = obj.Domain_;
        end
        
        
        function obj = set.Domain(obj, ~)
            LPVcore.error('LPVcore:general:PropertyNotInUse', 'Domain', 'lpviddata');
        end
        
        
        function val = get.ExperimentName(obj)
            val = obj.ExperimentName_;
        end
        
        
        function obj = set.ExperimentName(obj,val)
            
            if ~isstring(val) && (~ischar(val) || ~isvector(val))
                LPVcore.error('LPVcore:general:InvalidProperty', 'ExperimentName','1-by-n character array', 'lpviddata');
            else
                obj.ExperimentName_ = val;
            end
        end
        
        
        function val = get.InputData(obj)
            val = obj.InputData_;
        end
        

        function obj = set.InputData(obj,val)
            
            if isempty(val)
                obj.InputData_ = [];
                obj.OSISize(3) = 0;
                obj.InputName_ = cell(0,1);
                obj.InputUnit_ = cell(0,1);
                return;
            end
            
            if ~ismatrix(val) || ~isnumeric(val) || any(any(isnan(val))) || any(any(isinf(val))) || size(val,1) ~= obj.lengthDataSet
                LPVcore.error('LPVcore:general:InvalidProperty', 'InputData','a N-by-nu finite, real matrix', 'lpviddata');
            else


                obj.InputData_ = val;
                
                nuOld = obj.OSISize(3);
                nuNew = size(val,2);
                
                if obj.OSISize(3) ~= size(val,2)
                    obj.OSISize(3) = size(val,2);
                end
                
                if nuOld > nuNew
                    
                    obj.InputName_ = obj.InputName_(1:nuNew);
                    if ~isempty(obj.InputUnit_)
                        obj.InputUnit_ = obj.InputUnit_(1:nuNew);
                    end
                elseif nuOld < nuNew
                    
                    
                    Ntmp = cell(nuNew,1);
                    Ntmp(1:nuOld) = obj.InputName_;
                    for i = nuOld+1:nuNew
                        Ntmp{i} = localFindEmptyName('u', '', i-1, Ntmp(1:i));
                    end
                    obj.InputName_ = Ntmp;
                    
                    if ~isempty(obj.InputUnit_)
                        tmp = cell(nuNew - nuOld,1); tmp(:) = {''};
                        obj.InputUnit_ = [obj.InputUnit_; tmp];
                    end
                end
                
                
            end
            
        end
        
        
        function val = get.InputName(obj)
            val = obj.InputName_;
        end
        
        
        function obj = set.InputName(obj,InputName)
            
            if ~isempty(InputName)               
                if ~isstring(InputName) && ~ischar(InputName) && (~iscellstr(InputName) || ~isvector(InputName))
                    LPVcore.error('LPVcore:general:InvalidProperty','InputName', 'character array or cell array filled with unique character arrays','lpviddata');
                end
                
                if iscell(InputName) && length(unique(InputName)) ~= length(InputName)
                    LPVcore.error('LPVcore:general:InvalidProperty','InputName', 'character array or cell array filled with unique character arrays','lpviddata');
                end
            end

            if iscell(InputName) && isscalar(InputName); InputName = InputName{1,1}; % If 1x1 cell array, then unfold
            elseif ~ischar(InputName) && isrow(InputName); InputName = InputName'; end
            nu_ = size(InputName,1);
            
            nu = obj.OSISize(3); 

            if nu_ == nu
                if ~ischar(InputName); obj.InputName_ = InputName;
                else; obj.InputName_ = {InputName}; end
            elseif nu_ == 1 && nu_ ~= 0 % if scalar, create vector of names with indeces
                tmp_ = cell(nu,1);
                for i = 1:nu ; tmp_{i,1} = [InputName, num2str(i)]; end
                obj.InputName_ = tmp_;
            elseif nu_ == 0 || nu == 0  % Empty vector
                tmp_ = cell(nu,1);
                for i = 1:nu ; tmp_{i,1} = ['u', num2str(i)]; end
                obj.InputName_ = tmp_;
            else
                LPVcore.error('LPVcore:general:InvalidProperty','InputName', [sprintf('%d',nu),' x 1 cell array or character array'],'lpviddata');
            end
        end
        
        
        function val = get.InputUnit(obj)
            val = obj.InputUnit_;
        end
        
        function obj = set.InputUnit(obj,InputUnit)
            
            if ~isempty(InputUnit)

                if ~isstring(InputUnit) && ~ischar(InputUnit) && (~iscellstr(InputUnit) || ~isvector(InputUnit))
                    LPVcore.error('LPVcore:general:InvalidProperty', 'InputUnit', 'cell array or character array', 'lpviddata');
                end
            end
            
            if iscell(InputUnit) && isscalar(InputUnit); InputUnit = InputUnit{1,1}; % If 1x1 cell array, then unfold
            elseif ~ischar(InputUnit) && isrow(InputUnit); InputUnit = InputUnit'; end
            nu_ = size(InputUnit,1);
           
            nu = obj.OSISize(3);

            if nu_ == nu
                if ~ischar(InputUnit); obj.InputUnit_ = InputUnit;
                else; obj.InputUnit_ = {InputUnit}; end
            elseif nu_ == 1 && nu ~= 0 % if scalar, create vector of names with indeces
                tmp_ = cell(nu,1);
                tmp_(:,1) = {InputUnit};
                obj.InputUnit_ = tmp_;
            elseif nu_ == 0 || nu == 0% Empty vector
                tmp_ = cell(0,1);
                obj.InputUnit_ = tmp_;
            else
                LPVcore.error('LPVcore:general:InvalidProperty', 'InputUnit', [sprintf('%d',nu), ' x 1 cell array or character array'], 'lpviddata');
            end
        end
        
        
        function val = get.SchedulingData(obj)
            val = obj.SchedulingData_;
        end
        
        
        function obj = set.SchedulingData(obj,val)
            
            if ~ismatrix(val) || ~isnumeric(val) ...
                    || any(any(isnan(val))) || any(any(isinf(val))) ...
                    || size(val,2) ~= obj.OSISize(2) ...
                    || (~isempty(val) && size(val, 1) ~= obj.lengthDataSet)
                LPVcore.error('LPVcore:general:InvalidProperty', 'SchedulingData','a N-by-np finite, real matrix', 'lpviddata');
            else
                obj.SchedulingData_ = val;
                
                npOld = obj.OSISize(2);
                npNew = size(val,2);
                
                if obj.OSISize(2) ~= size(val,2)
                    obj.OSISize(2) = size(val,2);
                end
                
                if npOld > npNew
                    
                    obj.SchedulingName_ = obj.SchedulingName_(1:npNew);
                    if ~isempty(obj.SchedulingUnit_)
                        obj.SchedulingUnit_ = obj.SchedulingUnit_(1:npNew);
                    end
                elseif npOld < npNew
                    
                    
                    Ntmp = cell(npNew,1);
                    Ntmp(1:npOld) = obj.SchedulingName_;
                    for i = npOld+1:npNew
                        Ntmp{i} = localFindEmptyName('p', '', i-1, Ntmp(1:i));
                    end
                    
                    obj.SchedulingName_ = Ntmp;
                    
                    if ~isempty(obj.SchedulingUnit_)
                        tmp = cell(npNew - npOld,1); tmp(:) = {''};
                        obj.SchedulingUnit_ = [obj.SchedulingUnit_; tmp];
                    end
                end
            end
            
        end
        
        
        function val = get.SchedulingName(obj)
            val = obj.SchedulingName_;
        end
        
        
        function obj = set.SchedulingName(obj,SchedulingName)
            
            if ~isempty(SchedulingName)

                if ~isstring(SchedulingName) && ~ischar(SchedulingName) && (~iscellstr(SchedulingName) || ~isvector(SchedulingName))
                    LPVcore.error('LPVcore:general:InvalidProperty','SchedulingName', 'character array or cell array filled with unique character arrays','lpviddata');
                end
                
                if iscell(SchedulingName) && length(unique(SchedulingName)) ~= length(SchedulingName)
                    LPVcore.error('LPVcore:general:InvalidProperty','SchedulingName', 'character array or cell array filled with unique character arrays','lpviddata');
                end
            end

            if iscell(SchedulingName) && isscalar(SchedulingName); SchedulingName = SchedulingName{1,1}; % If 1x1 cell array, then unfold
            elseif ~ischar(SchedulingName) && isrow(SchedulingName); SchedulingName = SchedulingName'; end
            np_ = size(SchedulingName,1);
            
            np = obj.OSISize(2); 
            
            if np_ == np
                if ~ischar(SchedulingName); obj.SchedulingName_ = SchedulingName;
                else; obj.SchedulingName_ = {SchedulingName}; end
            elseif np_ == 1 && np_ ~= 0 % if scalar, create vector of names with indeces
                tmp_ = cell(np,1);
                for i = 1:np ; tmp_{i,1} = [SchedulingName, num2str(i)]; end
                obj.SchedulingName_ = tmp_;
            elseif np_ == 0 || np == 0  % Empty vector
                tmp_ = cell(np,1);
                for i = 1:np ; tmp_{i,1} = ['p', num2str(i)]; end
                obj.SchedulingName_ = tmp_;
            else
                LPVcore.error('LPVcore:general:InvalidProperty','SchedulingName', [sprintf('%d',np),' x 1 cell array or character array'],'lpviddata');
            end
        end
        
        
        function val = get.SchedulingUnit(obj)
            val = obj.SchedulingUnit_;
        end
        
        
        function obj = set.SchedulingUnit(obj,SchedulingUnit)
            
            if ~isempty(SchedulingUnit)

                if ~isstring(SchedulingUnit) && ~ischar(SchedulingUnit) && (~iscellstr(SchedulingUnit) || ~isvector(SchedulingUnit))
                    LPVcore.error('LPVcore:general:InvalidProperty', 'SchedulingUnit', 'cell array or character array of either of the following: ', 'lpviddata');
                end 
            end
            
            if iscell(SchedulingUnit) && isscalar(SchedulingUnit); SchedulingUnit = SchedulingUnit{1,1}; % If 1x1 cell array, then unfold
            elseif ~ischar(SchedulingUnit) && isrow(SchedulingUnit); SchedulingUnit = SchedulingUnit'; end
            np_ = size(SchedulingUnit,1);
           
            np = obj.OSISize(2); 

            if np_ == np
                if ~ischar(SchedulingUnit); obj.SchedulingUnit_ = SchedulingUnit;
                else; obj.SchedulingUnit_ = {SchedulingUnit}; end
            elseif np_ == 1 && np ~= 0 % if scalar, create vector of names with indeces
                tmp_ = cell(np,1);
                tmp_(:,1) = {SchedulingUnit};
                obj.SchedulingUnit_ = tmp_;
            elseif np_ == 0 || np == 0% Empty vector
                tmp_ = cell(0,1);
                obj.SchedulingUnit_ = tmp_;
            else
                LPVcore.error('LPVcore:general:InvalidProperty', 'SchedulingUnit', [sprintf('%d',np), ' x 1 cell array or character array'], 'lpviddata');
            end
        end
        
        
        function val = get.InterSample(~)
            val = [];
        end
        
        
        function obj = set.InterSample(obj, ~)
            LPVcore.error('LPVcore:general:PropertyNotInUse', 'InterSample', 'lpviddata');
        end
        
        
        function val = get.Name(obj)
            val = obj.Name_;
        end
        
        
        function obj = set.Name(obj,Name)
            
            if ~isempty(Name) && ( ~ischar(Name) || ~isvector(Name))
                LPVcore.error('LPVcore:general:InvalidProperty', 'Name','1-by-n character array', 'lpviddata');
            else
                obj.Name_ = Name;
            end
        end
        
        
        function val = get.Notes(obj)
            val = obj.Notes_;
        end
        
        
        function obj = set.Notes(obj,Notes)
            
            if ~isempty(Notes) ...
                    && ((~isstring(Notes) && ~ischar(Notes) && ~iscellstr(Notes)) || ~isvector(Notes))
                LPVcore.error('LPVcore:general:InvalidProperty', 'Notes','1-by-n character array or a cell vector of character arrays', 'lpviddata');
            end
            
            
            if iscolumn(Notes) && size(cell(0,1),1)>0; Notes = Notes'; end
                
            obj.Notes_ = Notes;
            
        end
        
        function val = get.OutputData(obj)
            val = obj.OutputData_;
        end
        
        
        function obj = set.OutputData(obj,OutputData)
            
            if isempty(OutputData)
                obj.OutputData_ = [];
                obj.OSISize(1) = 0;
                obj.OutputName_ = cell(0,1);
                obj.OutputUnit_ = cell(0,1);                
                return;
            end
            
            
            if ~ismatrix(OutputData) || ~isnumeric(OutputData) || any(any(isnan(OutputData))) || any(any(isinf(OutputData))) || size(OutputData,1) ~= obj.lengthDataSet
                LPVcore.error('LPVcore:general:InvalidProperty', 'OutputData','a N-by-ny finite, real matrix', 'lpviddata');
            else
                obj.OutputData_ = OutputData;
                
                nyOld = obj.OSISize(1);
                nyNew = size(OutputData,2);
                
                if obj.OSISize(1) ~= size(OutputData,2)
                    obj.OSISize(1) = size(OutputData,2);
                end
                
                if nyOld > nyNew
                    
                    obj.OutputName_ = obj.OutputName_(1:nyNew);
                    if ~isempty(obj.OutputUnit_)
                        obj.OutputUnit_ = obj.OutputUnit_(1:nyNew);
                    end
                elseif nyOld < nyNew
                    
                    Ntmp = cell(nyNew,1);
                    Ntmp(1:nyOld) = obj.OutputName_;
                    for i = nyOld+1:nyNew
                        Ntmp{i} = localFindEmptyName('y', '', i-1, Ntmp(1:i));
                    end
                    obj.OutputName_ = Ntmp;
                    
                    if ~isempty(obj.OutputUnit_)
                        tmp = cell(nyNew - nyOld,1); tmp(:) = {''};
                        obj.OutputUnit_ = [obj.OutputUnit_; tmp];
                    end
                end
            end
            
        end
        
        
        function val = get.OutputName(obj)
            val = obj.OutputName_;
        end
        
        
        function obj = set.OutputName(obj,OutputName)
            
            if ~isempty(OutputName)

                if ~isstring(OutputName) && ~ischar(OutputName) && (~iscellstr(OutputName) || ~isvector(OutputName))
                    LPVcore.error('LPVcore:general:InvalidProperty','OutputName', 'character array or cell array filled with unique character arrays','lpviddata');
                end
                
                if iscell(OutputName) && length(unique(OutputName)) ~= length(OutputName)
                    LPVcore.error('LPVcore:general:InvalidProperty','OutputName', 'character array or cell array filled with unique character arrays','lpviddata');
                end
            end

            if iscell(OutputName) && isscalar(OutputName); OutputName = OutputName{1,1}; % If 1x1 cell array, then unfold
            elseif ~ischar(OutputName) && isrow(OutputName); OutputName = OutputName'; end
            ny_ = size(OutputName,1);
            
            ny = obj.OSISize(1);

            if ny_ == ny
                if ~ischar(OutputName); obj.OutputName_ = OutputName;
                else; obj.OutputName_ = {OutputName}; end
            elseif ny_ == 1 && ny_ ~= 0 % if scalar, create vector of names with indeces
                tmp_ = cell(ny,1);
                for i = 1:ny ; tmp_{i,1} = [OutputName, num2str(i)]; end
                obj.OutputName_ = tmp_;
            elseif ny_ == 0 || ny == 0  % Empty vector
                tmp_ = cell(ny,1);
                for i = 1:ny ; tmp_{i,1} = ['y', num2str(i)]; end
                obj.OutputName_ = tmp_;
            else
                LPVcore.error('LPVcore:general:InvalidProperty','OutputName', [sprintf('%d',ny),' x 1 cell array or character array'],'lpviddata');
            end
        end
        
        
        function val = get.OutputUnit(obj)
            val = obj.OutputUnit_;
        end
        
        
        function obj = set.OutputUnit(obj,OutputUnit)
            
            if ~isempty(OutputUnit)

                if ~isstring(OutputUnit) && ~ischar(OutputUnit) && (~iscellstr(OutputUnit) || ~isvector(OutputUnit))
                    LPVcore.error('LPVcore:general:InvalidProperty', 'OutputUnit', 'cell array or character array', 'lpviddata');
                end            
            end
            
            if iscell(OutputUnit) && isscalar(OutputUnit); OutputUnit = OutputUnit{1,1}; % If 1x1 cell array, then unfold
            elseif ~ischar(OutputUnit) && isrow(OutputUnit); OutputUnit = OutputUnit'; end
            ny_ = size(OutputUnit,1);
           
            ny = obj.OSISize(1);

            if ny_ == ny
                if ~ischar(OutputUnit); obj.OutputUnit_ = OutputUnit;
                else; obj.OutputUnit_ = {OutputUnit}; end
            elseif ny_ == 1 && ny ~= 0 % if scalar, create vector of names with indeces
                tmp_ = cell(ny,1);
                tmp_(:,1) = {OutputUnit};
                obj.OutputUnit_ = tmp_;
            elseif ny_ == 0 || ny == 0% Empty vector
                tmp_ = cell(0,1);
                obj.OutputUnit_ = tmp_;
            else
                LPVcore.error('LPVcore:general:InvalidProperty', 'OutputUnit', [sprintf('%d',ny), ' x 1 cell array or character array'], 'lpviddata');
            end
        end
        
        
        function val = get.Period(obj)
            val = obj.Period_;
        end
        
        
        function obj = set.Period(obj,Period)
           
            if ~isscalar(Period) || ~isnumeric(Period) || isnan(Period) || ~isreal(Period) || Period <= 0
                LPVcore.error('LPVcore:general:InvalidProperty', 'Period', 'positive real number or inf', 'lpviddata');
            else
                obj.Period_ = Period;
            end
        end
        
        
        function val = get.TimeUnit(obj)
            val = obj.TimeUnit_;
        end
        
        
        function obj = set.TimeUnit(obj,TimeUnit)

            if ~isempty(TimeUnit) && (~isrow(TimeUnit) || ~ischar(TimeUnit))
                LPVcore.error('LPVcore:general:InvalidProperty','TimeUnit', ['charachter string of either of the following: ',strjoin(obj.unitsTime,', ')], 'lpviddata');
            end
            
            if any(strcmp(TimeUnit,obj.unitsTime))
                obj.TimeUnit_ = TimeUnit;
            elseif isempty(TimeUnit)
                obj.TimeUnit_ = 'seconds';
            else
                LPVcore.error('LPVcore:general:InvalidProperty','TimeUnit', ['charachter string of either of the following: ',strjoin(obj.unitsTime,', ')], 'lpviddata');
            end
        end
        
        
        function val = get.Ts(obj)
            val = obj.Ts_;
        end
        
        
        function obj = set.Ts(obj,Ts)
            
            if ~isscalar(Ts) || ~isnumeric(Ts) || ~isreal(Ts) || isnan(Ts) || isinf(Ts) || Ts <= 0
                LPVcore.error('LPVcore:general:InvalidProperty','Ts','number greater than 0', 'lpviddata');
            end
            
            obj.Ts_ = Ts;
        end
        
        
        function val = get.Tstart(obj)
            val = obj.Tstart_;
        end
        
 
        function obj = set.Tstart(obj,Tstart)
            
            if ~isscalar(Tstart) || ~isnumeric(Tstart) || ~isreal(Tstart) || isnan(Tstart) || isinf(Tstart)
                LPVcore.error('LPVcore:general:InvalidProperty','Tstart','real number', 'lpviddata');
            end
            
            obj.Tstart_ = Tstart;
        end
        

        function val = get.SamplingInstants(obj)
            N = obj.lengthDataSet;
            val = ((0:N-1)*obj.Ts + obj.Tstart )';
        end

        
        function val = get.y(obj)
            % y is alias for OutputData property
            val = obj.OutputData;
        end
        
        
        function obj = set.y(obj,val)
           try obj.OutputData = val; catch err; throw(err); end % Throws errors as caller function
        end
        
        function val = get.p(obj)
            % p is alias for SchedulingData
            val = obj.SchedulingData;
        end
        
        
        function obj = set.p(obj,val)
           try obj.SchedulingData = val; catch err; throw(err); end % Throws errors as caller function
        end
        
        
        function val = get.u(obj)
            % u is alias for InputData
            val = obj.InputData;
        end
        
        
        function obj = set.u(obj,val)
           try obj.InputData = val; catch err; throw(err); end % Throws errors as caller function
        end
        
        function val = get.t(obj)
            % t is alias for SamplingInstants
            val = obj.SamplingInstants;
        end
        
        
        function obj = set.t(obj,val)
           try obj.SamplingInstants = val; catch err; throw(err); end % Throws errors as caller function
        end
        
        [ySim, fit] = compare(data, sys, varargin);

    end % end of public methods
    
end



%-------------------- LOCAL FUNCTIONS -------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function name = localFindEmptyName(prefix, surfix, idxS, list)
    % Finds a name that is not contained in the list by incrementing a
    % number starting from idxS. Returns the name  [prefix,idx,surfix]
    
    for i = idxS+1:idxS+1e4
        name = [prefix,sprintf('%d',i),surfix];
        if ~any(strcmpi(name,list))
            break;
        end
    end
    
end
