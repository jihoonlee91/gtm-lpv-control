function bool = ispmatrix(obj,varargin)
%ISPMATRIX Determine if input is a pmatrix object
%
%   m = ispmatrix(A), returns logical 1 (true) if A is a pmatrix object.
%   Otherwise, it returns logical 0 (false).
%
%   m = ispmatrix(A,'affine'), returns logical 1 (true) if A is a pmatrix
%   object and all basis functions are either affine or parameter
%   independent. Otherwise, it returns logical 0 (false). 
%
%   m = ispmatrix(A,'poly'), returns logical 1 (true) if A is a pmatrix
%   object and all basis functions are either polynomials (including
%   affine) or parameter independent. Otherwise, it returns logical 0
%   (false).
%
%   m = ispmatrix(A,'custom'), returns logical 1 (true) if A is a pmatrix
%   object and all basis functions are either custom function or parameter
%   independent. Otherwise, it returns logical 0 (false).
%
%   See also PMATRIX

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    if ~isa(obj, 'pmatrix')
        bool = false;
    else
        bool = obj.ispmatrix(varargin{:}); % Require private property obj.typeBasis
    end

end

