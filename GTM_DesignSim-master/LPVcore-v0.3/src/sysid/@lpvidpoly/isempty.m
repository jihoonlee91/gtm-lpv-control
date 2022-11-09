function bool = isempty(obj)
%ISEMPTY Validates if the linear parameter-varying input-output
%representation with noise model is empty.
%
%   bool = isempty(obj)
%
% See also: LPVIDPOLY, LPVIDPOLY/SIZE, LPVIDPOLY/NDIMS.
%

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    if isequal(size(obj), [0 0])
        bool = true;
    else
        bool = false;
    end
end