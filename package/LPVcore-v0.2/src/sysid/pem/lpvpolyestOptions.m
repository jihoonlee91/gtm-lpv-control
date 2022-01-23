classdef (CaseInsensitiveProperties) lpvpolyestOptions < lpvsysidOptions
%LPVPOLYESTOPTIONS  Creates option set for the LPVPOLYEST command.
%
%   OPT = lpvpolyestOptions returns the default options for LPVPOLYEST.
%
%   OPT = lpvpolyestOptions('Option1',Value1,'Option2',Value2,...) uses name/value
%   pairs to override the default values for 'Option1','Option2',...
%
%   The supported options are:
%
%   SearchMethod   Choose between gradient-based estimation or
%       pseudo-linear least squares regression. Specify as one of:
%           'gradient': gradient-based estimation
%           'pseuols': pseudo-linear least squares
%       Default: 'gradient'
%
%   Initialization  Choose between initialization methods. Specify as one
%   of:
%           'template': initialization using the coefficients of the
%               provided template system.
%           'polypre': pre-estimation using an LPV-ARX or LPV-OE model
%               structure.
%           'riv': refined IV pre-estimation (SISO and LPV-{OE/BJ} model structures only).
%       Default: 'template'
%
%   Focus    Choose between 1-step prediction vs. simulation error
%            minimization. Specify as one of:
%             'prediction': Focus on producing good predictors. Default.
%
%   EstimateCovariance      Specify whether to estimate parameter covariance (true)
%                 or not (false). Default: true.
%
%   Verbose       Print estimation progress on command line (true/false).
%                   Default: false.
%
%   Display       View estimation progress and results ('on'/'off').
%                 Default: 'off'. 
%   
%
%   Regularization  Specify regularization scaling Lambda and weighting
%                 matrix R. Lambda must be zero (no regularization;
%                 default) or a positive scalar. R must be a positive
%                 semidefinite matrix of size equal to the number of model's
%                 free parameters (np = sum(sum([na nb nc]) for ARMAX models).
%                 Use the "armaxRegul" function to generate suitable values
%                 for Lambda and R.
%
%   SearchOptions   Structure with options for the iterative search
%   procedure. Fields:
%
%       FunctionTolerance: Termination tolerance on the loss function that 
%           the software minimizes to determine the estimated parameter 
%           values, specified as a positive scalar. Default: 1E-5
%
%       StepTolerance: Termination tolerance on the estimated parameter 
%           values, specified as a positive scalar. Default: 1E-6
%
%       MaxIterations: Maximum number of iterations during loss-function 
%           minimization, specified as a positive integer. The iterations 
%           stop when MaxIterations is reached or another stopping 
%           criterion is satisfied, such as FunctionTolerance. Default: 100
%
%   See also LPVARMAX, LPVARX, LPVOE, LPVBJ
%

    properties(Dependent)
        Advanced
    end
   
    properties
        SearchMethod  % 'grad' or 'pseudols'
        SearchOptions  % structure
        Initialization  % 'template', 'polypre', 'riv'
        Focus  % 'prediction' (currently unused)
        Verbose  % true / false
        Display   % progress display (on/off/full)
        EstimateCovariance
        Regularization
    end
    
    properties (Constant, Hidden)
        DefaultRegularization = struct('Lambda',0,'R',[],'OptionsUsed',[]);
        DefaultSearchOptions = struct('FunctionTolerance', 1E-5, ...
               'StepSize', 1, ...
               'StepTolerance', 1E-6, ...
               'MaxIterations', 100);
    end
   
    properties(Hidden)
        MaxSize
        ErrorThreshold  % LimitError
        LinearSolver    % Solver for inv(A)*b

        % Initial conditions/state handling option across various model types
        % and estimators. The allowable values are estimator dependent.
        IC_
        
        % Not yet implemented
        InputOffset
        SchedulingOffset
        OutputOffset
        OutputWeight
    end
   
   methods (Hidden, Access = protected)
      function Value = getAdvancedOptions_(this)
         % Create Advanced options as a structure
         Value = struct(...
            'ErrorThreshold',this.ErrorThreshold,...
            'MaxSize',this.MaxSize);
      end
      
      function this = setAdvancedOptions_(this,Value)
         % Update Advanced options
         
         if isfield(Value,'ErrorThreshold')
            this.ErrorThreshold = idoptions.utValidateEstimationOptions(...
               'ErrorThreshold',Value.ErrorThreshold);
         end
         
         this.MaxSize = idoptions.utValidateEstimationOptions(...
            'MaxSize',Value.MaxSize);
      end
      
      function Value = getCommandName(this)
         Value = this.CommandName;
      end
      
      function checkAdvancedfields(this, Value)
         % Check if Advanced property is a structure with valid fields.
         f = fieldnames(this.Advanced);
         if ~isstruct(Value) || ~isequal(sort(f),sort(fieldnames(Value)))
            Name = getCommandName(this);
            str = '';
            for ct = 1:numel(f)
               str = [str,f{ct},', ']; %#ok<AGROW>
            end
            ctrlMsgUtils.error('Ident:general:AdvOptionFields',Name, str(1:end-2));
         end
      end
     
   end % protected methods
   
   methods
       
       function this = lpvpolyestOptions(varargin)
           this.CommandName = 'lpvpolyest';
           this.SearchMethod = 'gradient';
           this.SearchOptions = this.DefaultSearchOptions;
           this.Regularization = this.DefaultRegularization;
           this.Initialization = 'template';
           this.Focus = 'prediction';
           this.Display = 'off';
           this.Verbose = false;
           this.EstimateCovariance = false;
           this.InputOffset = [];
           this.SchedulingOffset = [];
           this.OutputOffset = [];
           this.OutputWeight = [];
           this.IC_ = 'auto';
           this.MaxSize = 25e4;
           this.ErrorThreshold = 0;
           this.LinearSolver = @idpack.mldividecov;
           this = initOptions(this, varargin{:});
       end
       
        function this = set.SearchMethod(this, value)
            if ~strcmpi(value, {'gradient', 'pseudols'})
                error('SearchMethod must be ''gradient'' or ''pseudols''');
            end
            this.SearchMethod = lower(value);
        end
        
        function this = set.Initialization(this, value)
            if ~strcmpi(value, {'template', 'polypre', 'riv'})
                error('Initialization must be ''template'', ''polypre'' or ''riv''');
            end
            this.Initialization = lower(value);
        end
       
       function this = set.Focus(this, Value)
           if ~strcmpi(Value, 'prediction') && ~isempty(Value)
               LPVcore.error('LPVcore:general:InvalidProperty','Focus', 'string "prediction". This property is not used', 'lpvarxOptions');
           end
           this.Focus = 'prediction';
       end
       
       function this = set.Display(this,Display)
           this.Display = idoptions.utValidateEstimationOptions('Display', Display);
       end
       
       function this = set.EstimateCovariance(this, EstimateCovariance)
           assert(islogical(EstimateCovariance) && isscalar(EstimateCovariance), ...
               '''EstimateCovariance'' must be scalar logical');
           if ~isa(this, 'lpvarxOptions')
               assert(~EstimateCovariance, ...
                   '''EstimateCovariance'' is currently only supported for lpvarx');
           end
           this.EstimateCovariance = EstimateCovariance;
       end
       
       function this = set.InputOffset(this, InputOffset)
           InputOffset = idoptions.utValidateEstimationOptions('InputOffset', InputOffset);
           
           if ~isvector(InputOffset) && ~isempty(InputOffset); error(message('Ident:utility:IncorrectOffsetValue','InputOffset')); end
           
           this.InputOffset = InputOffset;
       end
       
       function this = set.SchedulingOffset(this, SchedulingOffset)

           if isnumeric(SchedulingOffset) && all(isfinite(SchedulingOffset(:))) && isvector(SchedulingOffset)
               this.SchedulingOffset = double(full(SchedulingOffset));
           elseif isempty(SchedulingOffset)
               this.SchedulingOffset = [];
           else
               error(message('Ident:utility:IncorrectOffsetValue','SchedulingOffset'))
           end
       end
       
       function this = set.OutputOffset(this, OutputOffset)
            OutputOffset = idoptions.utValidateEstimationOptions('OutputOffset', OutputOffset);
           
            if ~isvector(OutputOffset) && ~isempty(OutputOffset); error(message('Ident:utility:IncorrectOffsetValue','OutputOffset')); end
            
            this.OutputOffset = OutputOffset;
       end
       
       function this = set.OutputWeight(this, Value)
           
           if isempty(Value)
               this.OutputWeight = [];
           elseif isnumeric(Value)
               if ~idpack.isPosSemidef(Value)
                   error(message('Ident:estimation:invalidOutputWeight2'))
               end
               this.OutputWeight = Value;
           else
               LPVcore.error('LPVcore:general:InvalidProperty','OutputWeight', 'positive definite matrix', 'lpvarxOptions');
           end

       end
       
	function this = set.Regularization(this, Value)
         % SET method for "Regularization" property.
         if ~isstruct(Value)
            error(['The "Regularization" options must be a structure ', ...
                'with fields "R", "Lambda" and "OptionsUsed"'])
         end
         
         if ~isfield(Value, 'R') || isempty(Value.R)
             Value.R = [];
         else
         	[~,p] = chol(Value.R );
            if p ~= 0
                LPVcore.error('LPVcore:general:InvalidProperty','Regularization.R', 'positive definite matrix', 'lpvarxOptions');
            end
         end
         
         if ~isfield(Value, 'Lambda')
             Value.Lambda = 1;
         end
         
         if ~isnumeric(Value.Lambda) || ~isscalar(Value.Lambda) ...
                 || isnan(Value.Lambda) || isinf(Value.Lambda) || Value.Lambda < 0
             LPVcore.error('LPVcore:general:InvalidProperty','Regularization.Lambda', 'positive, real scalar', 'lpvarxOptions');
         end
         
         if ~isfield(Value, 'OptionsUsed')
             Value.OptionsUsed = [];
         end
         
         if ~isa(Value.OptionsUsed,'LPVcore.idoptions.lpvarxRegul') && ~isempty(Value.OptionsUsed)
             LPVcore.error('LPVcore:general:InvalidProperty','Regularization.OptionsUsed', 'lpvarxRegulOptions object or empty', 'lpvarxOptions');
         end
         
         this.Regularization = Value;
    end
      
    function this = set.SearchOptions(this, Value)
        % SET method for "SearchOptions" property.
        if ~isstruct(Value)
            error(['The "SearchOptions" options must be a structure ', ...
                'with fields "MaxIterations", "StepSize", "StepTolerance", and "FunctionTolerance"'])
        end

        if ~isfield(Value, 'MaxIterations')
            Value.MaxIterations = this.DefaultSearchOptions.MaxIterations;
        end
        
        if ~isfield(Value, 'StepSize')
            Value.StepSize = this.DefaultSearchOptions.StepSize;
        end
        
        if ~isfield(Value, 'StepTolerance')
            Value.StepTolerance = this.DefaultSearchOptions.StepTolerance;
        end
        
        if ~isfield(Value, 'FunctionTolerance')
            Value.FunctionTolerance = this.DefaultSearchOptions.FunctionTolerance;
        end
        
        this.SearchOptions = Value;
    end
       
       function Value = get.Advanced(this)
           Value = getAdvancedOptions_(this);
       end
       
       function this = set.Advanced(this, Value)
           % SET method for the Advanced property.
           checkAdvancedfields(this, Value);
           this = setAdvancedOptions_(this,Value);
       end
       
   end
   
   methods (Hidden)
       
       function varargout = display(this) %#ok<DISPLAY>
           [varargout{1:nargout}] = idoptions.displayOption(this,getCommandName(this));
       end
   end
   
end
