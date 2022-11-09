classdef (Abstract, CaseInsensitiveProperties) lpvsysidOptions
    %LPVSYSIDOPTIONS Base class for sys. id. options sets
    
    properties (Hidden)
        CommandName
    end
    
    methods (Hidden)
        function obj = initOptions(obj, varargin)
           if nargin == 1; return; end
           
           if mod(nargin-1,2) ~= 0
               LPVcore.error('LPVcore:general:InvalidNumOfInputs', class(obj));  
           end
           
           for i=1:2:nargin-1
               if ~ischar(varargin{i}) || ~isrow(varargin{i}) || ~isprop(obj,varargin{i})
                   LPVcore.error('LPVcore:general:UnknownPropertyValuePairs',varargin{i}, class(obj))
               end
               obj.(varargin{i}) = varargin{i+1};
           end
       end
    end
end

