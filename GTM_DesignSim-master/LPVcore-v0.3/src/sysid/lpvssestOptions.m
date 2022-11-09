classdef (CaseInsensitiveProperties) lpvssestOptions < lpvsysidOptions
%LPVSSESTOPTIONS  Creates option set for the LPVSSEST command.
%
%   OPT = lpvssestOptions returns the default options for lpvssest
%      estmation.
%
%   OPT = lpvssestOptions('Option1',Value1,'Option2',Value2,...) uses
%      name/value pairs to override the default values for
%      'Option1','Option2',...
%
%   The supported options are:
%
%   InitialState  Can be eiter 'zero' (not estimated from data) or
%                 'estimate'.
%
%   Focus    Choose between 1-step prediction vs. simulation error
%            minimization. Specify as one of:
%             'prediction': Focus on producing good predictors. Default.
%
%   Display       View estimation progress and results ('on'/'off').
%                 Default: 'off'. 
%
%   EstimateCovariance      Specify whether to estimate parameter covariance (true)
%                 or not (false). Covariance estimation is not yet supported. Default: false.
%   
%   InputOffset   Specifies the offset levels present in the input signals
%                 of the estimation data. Specify as an Nu-element column
%                 vector, where Nu is the number of inputs. Default is []
%                 which implies no offsets.
%
%   SchedulingOffset Scheduling signal offset levels. Similar
%                 interpretation as InputOffset. The specified values are
%                 subtracted from the scheduling signals before they are
%                 used for estimation.
%
%   OutputOffset  Output signal offset levels. Similar interpretation as
%                 InputOffset. The specified values are subtracted from the
%                 output signals before they are used for estimation.
%
%   OutputWeight  Weighting of prediction errors in multioutput
%                 estimations (irrelevant for single output estimations).
%                 Specify as a positive semi-definite matrix of size equal
%                 to the number of outputs. This amounts to minimizing
%                 trace(E'*E*W/N), where W is the specified matrix, E is
%                 the prediction error matrix and N is the number of
%                 samples. Default: [] implying Identity matrix for W.
%
%   SearchMethod  To select the nonlinear solver. The options are Enhanced
%                 Gauss-Newton solver 'egn', expectation-maximization 'em',
%                 nonlinear least squares solver 'lsqnonlin', and nonlinear
%                 solver 'fmincon'. Currently, only 'egn' is enabled. 
%                 For 'egn' and 'em' see [1]; and for
%                 lsqnonlin and fmincon type "help lsqnonlin" and "help
%                 fmincon", respectively.
%
%   SearchOptions  The options associated to the selected nonlinear solver
%                 defined in SearchMethod.
%
%
% [1] Cox, P. B. (2018). Towards Efficient Identification of Linear
% Parameter-Varying State-Space Models. Eindhoven University of Technology.
%
%
%   See also LPVSSEST

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).


   properties (Dependent)
      InitialState
      Focus
      Display   % progress display (on/off/full)
      EstimateCovariance
      InputOffset
      SchedulingOffset
      OutputOffset
      OutputWeight
      SearchMethod
      SearchOptions
   end
   
   properties (Access = protected)
      InitialState_ = 'estimate';
      Focus_ = 'prediction';
      Display_ = 'off';
      EstimateCovariance_ = false;
      InputOffset_ = [];
      SchedulingOffset_ = [];
      OutputOffset_ = [];
      OutputWeight_ = [];
      SearchMethod_ = 'egn';
      SearchOptions_ = lpvssestOptions.defaultOptionsegn;
   end
   
   
   properties (Constant, Hidden)
       defaultOptionsegn = struct('beta', 1e-4, 'gamma', 0.75, 'lambdaMIN', 0.00001, ...
           'alphaMIN', 0.001, 'nu', 0.01, 'epsilon', 0.000001, 'maxIter', 20, ...
           'FindSearchDirMaxIter',100, 'BackstepMaxIter',100, 'AbsTol', 1e-14, 'RelTol', 1e-15);
       
             % g indicates < or > and e indicates  >=  or <=
       validateOptionsegn = struct('beta',      struct('type','real', 'range',[0 0.5], 'bound', 'gg'), ...
                                   'gamma',     struct('type','real', 'range',[0 1], 'bound',  'ge'), ...
                                   'lambdaMIN', struct('type','real', 'range',[0 inf], 'bound',  'gg'), ...
                                   'alphaMIN',  struct('type','real', 'range',[0 inf], 'bound',  'gg'), ...
                                   'nu',        struct('type','real', 'range',[0 1], 'bound',  'gg'), ...
                                   'epsilon',   struct('type','real', 'range',[0 inf], 'bound',  'gg'), ...
                                   'maxIter',   struct('type','int',  'range',[1 inf], 'bound',  'eg'), ...
                                   'FindSearchDirMaxIter', struct('type','int',  'range',[1 inf], 'bound',  'eg'), ...
                                   'BackstepMaxIter',      struct('type','int',  'range',[1 inf], 'bound',  'eg'), ...
                                   'AbsTol',    struct('type','real',  'range',[0 inf], 'bound',  'gg'), ...
                                   'RelTol',    struct('type','real',  'range',[0 inf], 'bound',  'gg'));
       
       defaultOptionsem = struct('maxIter', 100, 'AbsTolFun', 1e-8, 'RelTolFun', 1e-10, ...
           'RelTolFunInit', 0.2, 'AbsTolParm', 1e-4, 'RelTolParm', 1e-3);
                               
       validateOptionsem = struct('maxIter',       struct('type','int',  'range',[1 inf], 'bound',  'eg'), ...
                                  'AbsTolFun',     struct('type','real',  'range',[0 inf], 'bound',  'gg'), ...
                                  'RelTolFun',     struct('type','real',  'range',[0 inf], 'bound',  'gg'), ...
                                  'RelTolFunInit', struct('type','real',  'range',[0 1], 'bound',  'ge'), ...
                                  'AbsTolParm',    struct('type','real',  'range',[0 inf], 'bound',  'gg'), ...
                                  'RelTolParm',    struct('type','real',  'range',[0 inf], 'bound',  'gg'));
       
       
       defaultOptionslsqnonlin = optimoptions('lsqnonlin','SpecifyObjectiveGradient',true);
       
       defaultOptionsfmincon = optimoptions('fmincon','GradObj','on');
   end
   
   
   methods (Access = protected)
      
      function Value = getCommandName(this)
         Value = this.CommandName;
      end
      
      function Value = checkSearchOptions(this, Value)
         % Check if SearchOptions property is a structure with valid fields.
         
         switch this.SearchMethod_
             case 'em'
                 if ~isequal( fieldnames(Value), ...
                         fieldnames(lpvssestOptions.defaultOptionsem) )
                     
                     LPVcore.error('LPVcore:general:InvalidProperty','SearchOptions', 'structure with correct fieldnames', 'lpvssestOptions');
                 end
                 
                 idx = localValidateFields(Value, lpvssestOptions.validateOptionsem);
                 
                 if idx ~= true
                     LPVcore.error('LPVcore:general:InvalidProperty',idx{1}, idx{2}, 'lpvssestOptions');
                 end
                 
             case 'lsqnonlin'
                 if ~isa(Value, 'optim.options.Lsqnonlin')
                     LPVcore.error('LPVcore:general:InvalidProperty','SearchOptions', 'lsqnonlin optionset', 'lpvssestOptions');
                 end
             case 'fmincon'
                 if ~isa(Value, 'optim.options.Fmincon')
                     LPVcore.error('LPVcore:general:InvalidProperty','SearchOptions', 'fmincon optionset', 'lpvssestOptions');
                 end
             otherwise
                 if ~isequal( fieldnames(Value), ...
                         fieldnames(lpvssestOptions.defaultOptionsegn) )
                     
                     LPVcore.error('LPVcore:general:InvalidProperty','SearchOptions', 'structure with correct fieldnames', 'lpvssestOptions');
                 end
                 
                 idx = localValidateFields(Value, lpvssestOptions.validateOptionsegn);
                 
                 if ~islogical(idx)
                     LPVcore.error('LPVcore:general:InvalidProperty',idx{1}, idx{2}, 'lpvssestOptions');
                 end
         end
         
      end
     
   end % protected methods
   
   methods
       
       function this = lpvssestOptions(varargin)
           this.CommandName = 'lpvssest';
           this = initOptions(this, varargin{:});
       end
       
       function Val = get.InitialState(this)
           Val = this.InitialState_;
       end
       
       function this = set.InitialState(this,Val)
           if ~any(strcmpi(Val, {'zero','estimate'}))
               LPVcore.error('LPVcore:general:InvalidProperty','InitialState', 'string with "zero" or "estimate"', 'lpvssestOptions');
           end
           
           this.InitialState_ = lower(Val);
       end
       
       function Val = get.Focus(this)
           Val = this.Focus_;
       end
       
       function this = set.Focus(this, Value)
           if ~strcmpi(Value, 'prediction') && ~isempty(Value)
               LPVcore.error('LPVcore:general:InvalidProperty','Focus', 'string "prediction". This property is not used', 'lpvssestOptions');
           end
           this.Focus_ = 'prediction';
       end
       
       function Val = get.Display(this)
           Val = this.Display_;
       end
       
       function this = set.Display(this,Display)
           this.Display_ = idoptions.utValidateEstimationOptions('Display', Display);
       end
       
       function Val = get.EstimateCovariance(this)
           Val = this.EstimateCovariance_;
       end
       
       function this = set.EstimateCovariance(this, EstCovar)
           assert(~EstCovar, ['''EstimateCovariance'' must be false since ', ...
               'covariance estimation is not yet supported']);
           this.EstimateCovariance_ = ...
               idoptions.utValidateEstimationOptions('EstimateCovariance', EstCovar);
       end
       
       function Val = get.InputOffset(this)
           Val = this.InputOffset_;
       end
       
       function this = set.InputOffset(this, InputOffset)
           InputOffset = idoptions.utValidateEstimationOptions('InputOffset', InputOffset);
           
           if ~isvector(InputOffset) && ~isempty(InputOffset); error(message('Ident:utility:IncorrectOffsetValue','InputOffset')); end
           
           this.InputOffset_ = InputOffset;
       end
       
       function Val = get.SchedulingOffset(this)
           Val = this.SchedulingOffset_;
       end
       
       function this = set.SchedulingOffset(this, SchedulingOffset)

           if isnumeric(SchedulingOffset) && all(isfinite(SchedulingOffset(:))) && isvector(SchedulingOffset)
               this.SchedulingOffset_ = double(full(SchedulingOffset));
           elseif isempty(SchedulingOffset)
               this.SchedulingOffset_ = [];
           else
               error(message('Ident:utility:IncorrectOffsetValue','SchedulingOffset'))
           end
       end
       
       
       function Val = get.OutputOffset(this)
           Val = this.OutputOffset_;
       end
       
       function this = set.OutputOffset(this, OutputOffset)
            OutputOffset = idoptions.utValidateEstimationOptions('OutputOffset', OutputOffset);
           
            if ~isvector(OutputOffset) && ~isempty(OutputOffset); error(message('Ident:utility:IncorrectOffsetValue','OutputOffset')); end
            
            this.OutputOffset_ = OutputOffset;
       end
       
       function Val = get.OutputWeight(this)
           Val = this.OutputWeight_;
       end
       
       function this = set.OutputWeight(this, Value)
           
           if isempty(Value)
               this.OutputWeight_ = [];
           elseif isnumeric(Value)
               if ~idpack.isPosSemidef(Value)
                   error(message('Ident:estimation:invalidOutputWeight2'))
               end
               this.OutputWeight_ = Value;
           else
               LPVcore.error('LPVcore:general:InvalidProperty','OutputWeight', 'positive definite matrix', 'lpvssestOptions');
           end

       end
       
       function Val = get.SearchMethod(this)
           Val = this.SearchMethod_;
       end
       
       function this = set.SearchMethod(this,Value)
           if ~any(strcmpi(Value, {'egn','em','lsqnonlin','fmincon'}))
               LPVcore.error('LPVcore:general:InvalidProperty','SearchMethod', 'string with "egn", "em", "lsqnonlin", or "fmincon"', 'lpvssestOptions');
           end
           if any(strcmpi(Value, {'em', 'lsqnonlin', 'fmincon'}))
                error('''%s'' is not yet enabled as a search method', Value);
           end
           
           if ~strcmp(Value,this.SearchMethod_)
               this.SearchMethod_  = lower(Value);
               
               switch this.SearchMethod_
                   case 'em'
                       if ~isequal( fieldnames(this.SearchOptions_), ... 
                               fieldnames(lpvssestOptions.defaultOptionsem) )
                           
                           this.SearchOptions_ = lpvssestOptions.defaultOptionsem;
                       end
                       
                   case 'lsqnonlin'
                       if ~isa(this.SearchOptions_, 'optim.options.Lsqnonlin')
                           this.SearchOptions_ = lpvssestOptions.defaultOptionslsqnonlin;
                       end
                   case 'fmincon'
                       if ~isa(this.SearchOptions_, 'optim.options.Fmincon')
                           this.SearchOptions_ = lpvssestOptions.defaultOptionsfmincon;
                       end
                   otherwise
                       if ~isequal( fieldnames(this.SearchOptions_), ... 
                               fieldnames(lpvssestOptions.defaultOptionsegn) )
                           
                           this.SearchOptions_ = lpvssestOptions.defaultOptionsegn;
                       end
               end
           end
       end
       
       function Val = get.SearchOptions(this)
           Val = this.SearchOptions_;
       end 
       
       function this = set.SearchOptions(this,Val)
             try Val = checkSearchOptions(this,Val);
             catch E; throw(E); end
            this.SearchOptions_ = Val;
       end   
       
       function varargout = display(this) %#ok<DISPLAY>
           [varargout{1:nargout}] = idoptions.displayOption(this,getCommandName(this));
       end
       
       function this = initOptions(this,varargin)           
           if nargin == 1; return; end         
           
           if mod(nargin-1,2) ~= 0
               LPVcore.error('LPVcore:general:InvalidNumOfInputs', 'lpvssestOptions');  
           end
           
           for i = 1:2:nargin-1
               if ~ischar(varargin{i}) || ~isrow(varargin{i}) || ~isprop(this,varargin{i})
                   LPVcore.error('LPVcore:general:UnknownPropertyValuePairs',varargin{i},'lpvssestOptions')
               end
               this.(varargin{i}) = varargin{i+1};
           end
       end

   end
   
end

function bool = localValidateFields(f, options)

    bool = true;
    fnames = fieldnames(f);

    for i = 1:numel(fnames)

        val = f.(fnames{i});

        lb = strcmp('e', options.(fnames{i}).bound(1));
        ub = strcmp('e', options.(fnames{i}).bound(2));

        switch options.(fnames{i}).type
            
            case 'real'

                if ~isnumeric(val) || ~isscalar(val) || ~isreal(val)
                    bool = false;
                else
                    if lb && val < options.(fnames{i}).range(1)
                        bool = false;
                    elseif ~lb && val <= options.(fnames{i}).range(1)
                        bool = false;
                    end

                    if ub && val > options.(fnames{i}).range(2)
                        bool = false;
                    elseif ~ub && val >= options.(fnames{i}).range(2)
                        bool = false;
                    end
                end
                
                if bool == false    % create error message
                    
                    if lb; lbStr = ' <= '; else;  lbStr = ' < '; end
                    if ub; ubStr = ' <= '; else;  ubStr = ' < '; end
                    
                    bool = {['SearchOptions.',fnames{i}], ['real (',num2str(options.(fnames{i}).range(1)),lbStr,'x',ubStr,num2str(options.(fnames{i}).range(2)),')']};
                    return;
                end
                
            case 'int'

                if ~isnumeric(val) || ~isscalar(val) || round(val) ~= val
                    bool = false;
                else
                    if lb && val < options.(fnames{i}).range(1)
                        bool = false;
                    elseif ~lb && val <= options.(fnames{i}).range(1)
                        bool = false;
                    end

                    if ub && val > options.(fnames{i}).range(2)
                        bool = false;
                    elseif ~ub && val >= options.(fnames{i}).range(2)
                        bool = false;
                    end
                end
                
                if bool == false    % create error message
                    
                    if lb; lbStr = ' <= '; else;  lbStr = ' < '; end
                    if ub; ubStr = ' <= '; else;  ubStr = ' < '; end
                    
                    bool = {['SearchOptions.',fnames{i}], ['integer ( ',num2str(options.(fnames{i}).range(1)),lbStr,'x',ubStr,num2str(options.(fnames{i}).range(2)),' )']};
                    return;
                end
        
        end
    end

end