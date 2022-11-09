classdef (CaseInsensitiveProperties) lpvarmaxOptions < lpvpolyestOptions
% Class definition for the LPV ARX estimation class.

   methods
       
       function this = lpvarmaxOptions(varargin)
           this = this@lpvpolyestOptions(varargin{:});
           this.CommandName = 'lpvarmax';
       end

   end
   
end
