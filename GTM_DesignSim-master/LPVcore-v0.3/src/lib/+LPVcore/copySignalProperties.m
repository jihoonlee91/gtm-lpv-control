function target = copySignalProperties(target, source, mode)
%COPYSIGNALPROPERTIES Copy signal properties from source system to target
%system
%
% This function can be used to conveniently map signal properties (such as
% input/output name and unit) from a source system to a target system.
%
% Syntax:
%   sys = LPVcore.copySignalProperties(target, source)
%   sys = LPVcore.copySignalProperties(target, source, options)
%
% Inputs:
%   target: dynamical system for which signal properties (inputs and
%       outputs) are copied from 'source'.
%   source: dynamical system from which signal properties (inputs and
%       outputs) are copied to 'target'.
%   options: set to 'full' to also include signal properties of the state
%       vector. By default or when set to 'io', only the input/output signal properties are
%       copied.
%

%% Input validation
narginchk(2, 3);
assert(isequal(size(target), size(source)), ...
    '''target'' and ''source'' must have same IO dimension');
if nargin <= 2
    mode = 'io';
end
assert(any(strcmp(mode, {'io', 'full'})), ...
    ['Unsupported copy mode ''', mode, '''. Supported options are ''io'' or ''full''']);
% Check state size for 'full' mode
if strcmp(mode, 'full')
    assert(isequal(size(target.StateName), size(source.StateName)), ...
        '''source'' and ''target'' must have matching state dimension for mode=''full''');
end

%% Properties to copy
properties = {'InputName', 'OutputName', ...
                'InputUnit', 'OutputUnit', ...
                'InputDelay'};
% Add state properties depending on mode
if strcmp(mode, 'full')
    properties{end+1} = 'StateName';
    properties{end+1} = 'StateUnit';
end
% Perform the copy
for i=1:numel(properties)
    % Check if both systems have the property, as this may not always be
    % the case. For example, if s1 is a TF model and s2 is a SS model, then
    % s1 does not have StateName defined.
    if isprop(source, properties{i}) && isprop(target, properties{i})
        target.(properties{i}) = source.(properties{i});
    end
end

end

