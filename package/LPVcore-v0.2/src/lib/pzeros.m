function P = pzeros(varargin)
%PZEROS Create array of zero pmatrix

mat = zeros(varargin{:});
P = double2pmatrix(mat);

end

