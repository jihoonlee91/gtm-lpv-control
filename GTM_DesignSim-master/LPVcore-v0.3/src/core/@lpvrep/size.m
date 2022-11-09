function varargout = size(obj, varargin)
% SIZE The IO size of the LPV system representation
%
%   D = size(A), returns the two-element row vector D = [ny,nu] containing
%   the number of outputs and inputs of the system representation.
%
%   [m,n] = SIZE(A), returns the number of outputs and inputs in A as
%   separate output variables.
%
%   D = SIZE(A,dim) returns the length of the dimension specified by the
%   scalar dim. If DIM > 3, D will be 1.
%
% See also: SIZE, LPVIDPOLY/NDIMS.
%

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 

    sz = [obj.Ny, obj.Nu];

    if nargin == 2
        dim = varargin{1};
        if (dim > 2)
            varargout{1} = 1;
        else
            varargout{1} = sz(dim);
        end
    elseif nargout <= 1
        varargout{1} = sz;
    elseif nargout == 2
        varargout{1} = sz(1);
        varargout{2} = sz(2);
    end
end