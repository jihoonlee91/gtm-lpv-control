function value = get(obj,varargin)
%GET  Access/query property values of lpviddata.
%
%   VALUE = get(M,'PropertyName') returns the value of the specified
%   property of the input/output model M. An equivalent syntax is VALUE =
%   M.PropertyName.
%
%   VALUES = get(M,{'Prop1','Prop2',...}) returns the values of several
%   properties at once in a cell array VALUES.
%
%   get(M) displays all properties of M and their values.
%
%   S = get(M) returns a structure whose field names and values are the
%   property names and values of M.
%
%   See also TIMEMAP/SET, TIMEMAP.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).
    
    try
        value = LPVcore.get(obj,varargin{:});
    catch ME
        throw(ME);
    end

    if nargout == 0 && nargin == 1
        disp(value);
        clear value;
    end
end