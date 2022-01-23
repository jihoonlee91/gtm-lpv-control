function P = peye(varargin)
%PZEROS Create array of identity pmatrix

mat = eye(varargin{:});
P = double2pmatrix(mat);

end

