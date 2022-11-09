classdef lpvarxRegul 
% Class definition for the LPV ARX regularization class.
    
% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

   properties
      InputOffset
      SchedulingOffset
      OutputOffset
      RegulKernel
      Advanced
   end
   
	properties (Access = protected)
      CommandName
   end
   
   
   methods
      function this = lpvarxRegul()
         % Constructor.
         this.RegulKernel = 'eye';
         this.InputOffset = [];
         this.SchedulingOffset = [];
         this.OutputOffset = [];
         this.CommandName = 'lpvarxRegul';
         this.Advanced = struct('MaxSize',250e3,'SearchMethod','fmincon');
      end
      
      function this = set.RegulKernel(this, Value)
         % SET method for Kernel property.
         Possibilities = {'eye'};
         Value = ltipack.matchKey(Value,Possibilities);
         if isempty(Value)
            error(message('Ident:estimation:arxRegulCheck3'))
         end
         this.RegulKernel = Value;
      end
      
      function this = set.Advanced(this, Value)
         % SET method for "Advanced" property.
         if ~isstruct(Value) || ~isequal(sort(fieldnames(Value)),...
               sort({'MaxSize';'SearchMethod'}))
            error(message('Ident:estimation:arxRegulOptionsChk1'))
         end
         Value.MaxSize = idoptions.utValidateEstimationOptions(...
            'MaxSize',Value.MaxSize);
         SM = ltipack.matchKey(Value.SearchMethod,{'fmincon','gn'});
         if isempty(SM)
            error(message('Ident:estimation:impulseestOptionsChk4'))
         else
            Value.SearchMethod = SM;
         end
         this.Advanced = Value;
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
       
       function varargout = display(this) %#ok<DISPLAY>
           [varargout{1:nargout}] = idoptions.displayOption(this,getCommandName(this));
       end
       
       function this = initOptions(this,varargin)
           
           if iscell(varargin) && isvector(varargin)
               varargin = varargin{:};
               nargin = numel(varargin)+1;
           end
           
           if nargin == 1; return; end
           
           if mod(nargin-1,2) ~= 0
               LPVcore.error('LPVcore:general:InvalidNumOfInputs', 'lpvarxOptions');
           end
           
           for i = 1:2:nargin-1
               if ~ischar(varargin{i}) || ~isrow(varargin{i}) || ~isprop(this,varargin{i})
                   LPVcore.error('LPVcore:general:UnknownPropertyValuePairs',varargin{i},'lpvarxRegulOptions')
               end
               this.(varargin{i}) = varargin{i+1};
           end
       end
       
   end
   
   methods (Access = protected)
      function Value = getCommandName(this)
         Value = this.CommandName;
      end
   end
end
