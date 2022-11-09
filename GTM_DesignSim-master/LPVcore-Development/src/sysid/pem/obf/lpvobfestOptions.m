classdef (CaseInsensitiveProperties) lpvobfestOptions < lpvsysidOptions
%LPVOBFESTOPTIONS  Creates option set for the LPVOBFESTOPTIONS command.
%
%   OPT = lpvobfestOptions returns the default options for LPVOBFEST.
%
%   OPT = lpvobfestOptions('Option1',Value1,'Option2',Value2,...) uses name/value
%   pairs to override the default values for 'Option1','Option2',...
%
%   The supported options are:
%
%   Display       View estimation progress and results ('on'/'off').
%                 Default: 'off'. 
%   
%
%   Regularization  Specify regularization scaling Lambda and weighting
%                 matrix R. Lambda must be zero (no regularization;
%                 default) or a positive scalar. R must be a positive
%                 semidefinite matrix of size equal to the number of model's
%                 free parameters.
%
%   InitialState  Handling of initial states during estimation, specified
%       as one of the following values:
%           * 'zero': the initial state is set to zero (default).
%           * 'estimate': the initial state is treated as an independent
%                   estimation parameter (Hammerstein only).
%           * Vector of doubles: the specified values are treated as the
%               fixed initial state during the estimation process.
%
%   See also LPVOBFEST
%
   
    properties
        Display   % progress display (on/off/full)
        Regularization
        InitialState
    end
    
    properties (Constant, Hidden)
        DefaultRegularization = struct('Lambda',0,'R',[],'OptionsUsed',[]);
    end
   
   methods (Hidden, Access = protected)
      
      function Value = getCommandName(this)
         Value = this.CommandName;
      end
     
   end % protected methods
   
   methods
       
       function this = lpvobfestOptions(varargin)
           this.CommandName = 'lpvobfest';
           this.Regularization = this.DefaultRegularization;
           this.Display = 'off';
           this.InitialState = 'zero';
           this = initOptions(this, varargin{:});
       end
       
       function this = set.Display(this,Display)
           this.Display = idoptions.utValidateEstimationOptions('Display', Display);
       end
       
        function this = set.InitialState(this, Value)
            if ischar(Value)
                if ~ismember(Value, {'zero', 'estimate'})
                    error('Invalid options for ''InitialState''');
                end
            elseif ~(iscolumn(Value) && isnumeric(Value))
                error('''InitialState'' must be a column vector of doubles');
            end
            this.InitialState = Value;
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
   end
   
   methods (Hidden)
       
       function varargout = display(this) %#ok<DISPLAY>
           [varargout{1:nargout}] = idoptions.displayOption(this,getCommandName(this));
       end

       function x0 = getInitialState(this, template_sys)
            if isequal(this.InitialState, 'zero')
                x0 = zeros(template_sys.Nx, 1);
            elseif isnumeric(this.InitialState)
                x0 = this.InitialState;
            else
                x0 = NaN(template_sys.Nx, 1);
            end
       end
   end
   
end
