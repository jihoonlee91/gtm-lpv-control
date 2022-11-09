function sys = lpvoe(data, template_sys, options)
%LPVOE Extimating an LPV-OE model from data    
%
%   Syntax:
%       sys = lpvoe(data, template_sys)
%       sys = lpvoe(data, template_sys, options)
%
%   Inputs:
%       data: lpviddata object.
%       template_sys: template lpvidpoly object. The structure will be
%           used, but not the coefficient values.
%       options: lpvoeOptions for controlling the identification
%           parameters.
%
%   Outputs:
%       sys: estimated model.
%
% See also PMATRIX, LPVOEOPTIONS
narginchk(2, 3);

assert(isa(data, 'lpviddata'), ...
    'Data must be of type lpviddata.');
assert(isa(template_sys, 'lpvidpoly'), ...
    'Template system must be an lpvidpoly object.');
assert(isoe(template_sys), ...
    'Template system must be an LPV-OE model structure.');
[~,~,~,nu] = size(data);
assert(nu > 0, 'Data must have at least 1 input');

if nargin <= 2
    options = lpvoeOptions;
end

assert(isa(options, 'lpvpolyestOptions'), 'options must be an instance of lpvoeOptions');

sys = lpvpolyest(data,template_sys,options);

end