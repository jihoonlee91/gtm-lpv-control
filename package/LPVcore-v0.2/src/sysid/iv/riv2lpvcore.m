function sys = riv2lpvcore(sys, Ae, Be, Ce, De)
%RIV2LPVCORE Summary of this function goes here
%   Detailed explanation goes here

sys.F = rivmat2lpvidfilt(sys.F, Ae, 2);
sys.B = rivmat2lpvidfilt(sys.B, Be, 1);
%%TODO: Ce and De
sys.C = rivmat2lpvidfilt(sys.C, Ce, 2);
sys.D = rivmat2lpvidfilt(sys.D, De, 2);

end

function A = rivmat2lpvidfilt(A, mat, k)
    % A: LPVIDFILT object
    % mat: the RIV matrix representation
    % k: the start index (set to 2 for monic filters)
    if size(mat, 1) ~= A.N
        error('Trying to convert RIV matrix to lpvidfilt object, but order is incompatible.');
    end
    for i=k:size(mat, 1)
        A.Mat{i} = rivrow2pmatrix_(A.Mat{i}, mat(i, :));
    end
end

function A = rivrow2pmatrix_(A, row)
    A.matrices = reshape(row, [1, 1, A.numbfuncs]);
end

