function obj = lpvss2lpvidss(obj,syslpvss)
%LPVSS2LPVIDSS Transform lpvss object to lpvidss object
%
%   Syntax:
%   sys = lpvidss.lpvss2lpvidss(syslpvss)
%
%   syslpvss is an lpvss object.
%   sys is an lpvidss object.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    if nargin == 1; return; end
    
    obj = lpvidss;
    
    if ~isa(syslpvss, 'lpvss')
        LPVcore.error('LPVcore:general:InvalidInputArgument','second','lpvss object','lpvidss/private/lpvss2lpvidss');
    end

    
    obj.Ts             = syslpvss.Ts;
    obj.A              = syslpvss.A;
    obj.B              = syslpvss.B;
    obj.C              = syslpvss.C;
    obj.D              = syslpvss.D;

    obj.StateName       = syslpvss.StateName;
    obj.StateUnit       = syslpvss.StateUnit;
    obj.SchedulingName  = syslpvss.SchedulingName;
    obj.SchedulingUnit  = syslpvss.SchedulingUnit;
    obj.NoiseVariance   = 1;
    obj.InputDelay_     = syslpvss.InputDelay;
    obj.TimeUnit_       = syslpvss.TimeUnit;
    obj.InputName_      = syslpvss.InputName;
    obj.InputUnit_      = syslpvss.InputUnit;
    obj.InputGroup_     = syslpvss.InputGroup;
    obj.OutputName_     = syslpvss.OutputName;
    obj.OutputUnit_     = syslpvss.OutputUnit;
    obj.OutputGroup_    = syslpvss.OutputGroup;
    obj.Name_           = syslpvss.Name;
    obj.Notes_          = syslpvss.Notes;
    obj.UserData        = syslpvss.UserData;

    
end


