function bool = isempty(obj)
%ISEMPTY Validates if the linear parameter-varying state-space
%representation with noise model is empty.
%
%   bool = isempty(obj)
%
% See also: LPVIDSS, LPVIDSS/SIZE, LPVIDSS/NDIMS.
%

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    if obj.Ny == 0 && obj.Nu == 0 && obj.Nx == 0
        bool = true;
    else
        bool = false;
    end
end