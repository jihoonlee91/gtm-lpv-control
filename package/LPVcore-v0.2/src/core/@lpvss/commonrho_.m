function [varargout] = commonrho_(varargin)
%COMMONRHO_ Reformulates input lpvss objects to share common ext.
%scheduling parameter.
%
%   Syntax:
%       [B1, ..., Bn] = commonrho_(A1, ..., An)
%       [B1, ..., Bn, tm] = commonrho_(A1, ..., An)
%
%   Inputs:
%       A1, ..., An: lpvss objects.
%
%   Outputs:
%       B1, ..., Bn: lpvss objects sharing same timemap.
%       tm: timemap
%

    if nargin ~= nargout
        error('Number of output arg. must be number of input args.');
    end

    % Convert input arguments to lpvss objects
    for i=1:nargin
        if ~isa(varargin{i}, 'lpvss')
            varargin{i} = lpvss(varargin{i});
        end
    end

    % Find representation with common rho
    SCommon = varargin{1};
    for i=2:nargin
        [SCommon, ~] = commonrho__(SCommon, varargin{i});
    end

    % Apply representation with common rho
    varargout = cell(1, nargout);
    for i=1:nargin
        [~, varargout{i}] = commonrho__(SCommon, varargin{i});
    end

end

function [S1, S2] = commonrho__(S1, S2)
    % COMMONRHO__ Local function to convert two systems to common rho
    % representation.
    [S1.A_, S1.B_, S1.C_, S1.D_, S2.A_, S2.B_, S2.C_, S2.D_] = ...
            pmatrix.commonrho_(S1.A_, S1.B_, S1.C_, S1.D_, S2.A_, S2.B_, S2.C_, S2.D_);
end

