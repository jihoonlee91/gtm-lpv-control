function [varargout] = commonrho_(varargin)
%COMMONRHO_ Reformulates input pmatrix objects to share common ext.
%scheduling parameter.
%
%   Syntax:
%       [B1, ..., Bn] = commonrho_(A1, ..., An)
%       [B1, ..., Bn, tm] = commonrho_(A1, ..., An)
%
%   Inputs:
%       A1, ..., An: pmatrix objects.
%
%   Outputs:
%       B1, ..., Bn: pmatrix objects sharing same timemap.
%       tm: timemap
%

    if nargin ~= nargout && nargin + 1 ~= nargout
        error('LPVcore:pmat:inOutMismatch',...
            'Number of output arg. must be number of input args + {1/0}.');
    end
    
    % Convert input arguments to pmatrix objects (if not already pmatrix)
    for i=1:nargin
        varargin{i} = obj2pmatrix(varargin{i});
    end

    tm = cell(1, nargin);
    for i=1:nargin
        tm{i} = varargin{i}.timemap;
    end
    
    if nargin == 0
        varargout{1} = tm;
        return;
    end
    
    tmCommon = tm{1};
    for i=2:nargin
        tmCommon = union(tmCommon, tm{i});
    end
    
    for i=1:nargin
        varargin{i}.timemap = tmCommon;
    end
    
    if nargout == nargin
        varargout = varargin;
    else
        varargout(1:nargin) = varargin;
        varargout{nargout} = tmCommon;
    end

end

