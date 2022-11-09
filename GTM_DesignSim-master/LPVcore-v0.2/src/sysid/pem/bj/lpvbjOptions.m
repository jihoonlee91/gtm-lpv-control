classdef (CaseInsensitiveProperties) lpvbjOptions < lpvpolyestOptions
% Class definition for the LPV BJ estimation class.

   methods
       
       function this = lpvbjOptions(varargin)
           this = this@lpvpolyestOptions(varargin{:});
           this.CommandName = 'lpvbj';
       end

   end
   
end
