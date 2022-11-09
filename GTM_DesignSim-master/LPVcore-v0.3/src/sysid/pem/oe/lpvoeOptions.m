classdef (CaseInsensitiveProperties) lpvoeOptions < lpvpolyestOptions
% Class definition for the LPV BJ estimation class.

   methods
       
       function this = lpvoeOptions(varargin)
           this = this@lpvpolyestOptions(varargin{:});
           this.CommandName = 'lpvoe';
       end

   end
   
end
