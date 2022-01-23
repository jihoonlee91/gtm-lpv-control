function obj = set(obj,varargin)
%SET  Modifies values of lpviddata properties.
%
%   OBJ = set(OBJ,'Property',VALUE) sets the property with name 'Property'
%   to the value VALUE. This is equivalent to OBJ.Property = VALUE.
%
%   OBJ = set(OBJ,'Property1',Value1,'Property2',Value2,...) sets multiple
%   property values in a single command.
%
%   OBJOUT = set(OBJ,'Property1',Value1,...) returns the modified
%   object OBJOUT.
%
%   See also LPVIDDATA/GET, LPVIDDATA.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    try   % Error is thrown as the class constructor function not LPVcore.set
        obj = LPVcore.set(obj,varargin{:});
    catch ME
        throw(ME);
    end
end