classdef (CaseInsensitiveProperties) lpvarxOptions < lpvpolyestOptions
% Class definition for the LPV ARX estimation class.

    properties (Constant, Hidden, Access=protected)
        unusedOptions = {'SearchMethod', 'SearchOptions'};
    end

   methods
       function this = lpvarxOptions(varargin)
           this = this@lpvpolyestOptions(varargin{:});
           this.CommandName = 'lpvarx';
       end
   end
   
    methods (Hidden)
        function this = initOptions(this,varargin)
            this = initOptions@lpvpolyestOptions(this, varargin{:});
            % Check whether unused options were passed
            if numel(varargin) > 0
                specifiedOptions = varargin{1:2:numel(varargin)-1};
                for i=1:numel(this.unusedOptions)
                    if any(strcmpi(this.unusedOptions{i}, specifiedOptions))
                        warning('%s is unused for lpvarx', ...
                            this.unusedOptions{i});
                    end
                end
            end
        end
    end
   
end
