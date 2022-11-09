function [varargout] = commonrho_(varargin)
%COMMONRHO_ Reformulates input lpvlfr objects to share common ext.
%scheduling variable.
%
%   Syntax:
%       [B1, ..., Bn] = commonrho_(A1, ..., An)
%       [B1, ..., Bn, tm] = commonrho_(A1, ..., An)
%
%   Inputs:
%       A1, ..., An: lpvlfr objects.
%
%   Outputs:
%       B1, ..., Bn: lpvlfr objects sharing same timemap.
%       tm: timemap
%

    if nargin ~= nargout
        error('Number of output arg. must be number of input args.');
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
    [S1.Delta_,S2.Delta_] = pmatrix.commonrho_(S1.Delta_,S2.Delta_);
end
