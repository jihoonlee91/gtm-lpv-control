function Value = findProp(Prop, varg, PVstart, Value)
% FINDPROP searches for the property value in varargin for LPVcore classes.
%
% Value = findProp(Prop, varg, PVstart, ValueDefault) searches and returns
% the value of the property "Prop" in varargin "varg". It starts searching
% from PVstart index. If the property is not contained in varargin then
% Value in the calling argument is returned.
%

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    if isempty(PVstart), return, end
    Loc = find(strcmpi(varg(PVstart:2:end),Prop));
    if ~isempty(Loc)
       Loc = 2*Loc + PVstart-2;
       Value = varg{Loc(end)+1};
    end
end