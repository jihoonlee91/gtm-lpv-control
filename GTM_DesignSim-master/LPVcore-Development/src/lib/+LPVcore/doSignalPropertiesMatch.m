function tf = doSignalPropertiesMatch(s1, s2, mode)
%DOSIGNALPROPERTIESMATCH Check whether signal properties of two systems
%match.
%
% This function can be used to conveniently check whether signal properties (such as
% input/output name and unit) are equal for 2 given systems.
%
% Syntax:
%   tf = LPVcore.doSignalPropertiesMatch(s1, s2)
%   tf = LPVcore.doSignalPropertiesMatch(s1, s2, options)
%
% Inputs:
%   s1, s2: dynamical system for which signal properties (inputs and
%       outputs) are checked for equivalence.
%   options: set to 'full' to also include signal properties of the state
%       vector. By default or when set to 'io', only the input/output signal properties are
%       checked.
%

%% Input validation
narginchk(2, 3);
assert(isequal(size(s1), size(s2)), ...
    '''s1'' and ''s2'' must have same IO dimension');
if nargin <= 2
    mode = 'io';
end
assert(any(strcmp(mode, {'io', 'full'})), ...
    ['Unsupported mode ''', mode, '''. Supported options are ''io'' or ''full''']);
% Check state size for 'full' mode
if strcmp(mode, 'full')
    assert(isequal(size(s1.StateName), size(s2.StateName)), ...
        '''s1'' and ''s2'' must have matching state dimension for mode=''full''');
end

%% Properties to copy
properties = {'InputName', 'OutputName', ...
                'InputUnit', 'OutputUnit', ...
                'InputDelay', 'Ts'};
% Add state properties depending on mode
if strcmp(mode, 'full')
    properties{end+1} = 'StateName';
    properties{end+1} = 'StateUnit';
end
% Perform the check
tf = true;
for i=1:numel(properties)
    % Check if both systems have the property, as this may not always be
    % the case. For example, if s1 is a TF model and s2 is a SS model, then
    % s1 does not have StateName defined.
    if isprop(s1, properties{i}) && isprop(s2, properties{i})
        if ~isequal(s1.(properties{i}), s2.(properties{i}))
            tf = false;
            return;
        end
    end
end

end



