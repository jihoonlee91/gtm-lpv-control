classdef pbaffine < pbpoly
    %@PBAFFINE pbasis function affine in rho.
    
    methods
        function obj = pbaffine(idx)
            %PBAFFINE Constructs pbaffine basis function.
            %   Syntax:
            %       p = pbaffine(m, idx)
            %
            %   Inputs:
            %       m (int): dimension of rho.
            %       idx (int): index of rho appearing in the affine expression.
            %
            degrees = zeros(1, idx);
            if idx > 0
                degrees(idx) = 1;
            end
            obj = obj@pbpoly(degrees);
            obj.dependenceType = 'affine';
        end
        
        function C = times(A, B)
            if isa(A, 'pbaffine') && isa(B, 'pbconst')
                C = A;
            elseif isa(A, 'pbconst') && isa(B, 'pbaffine')
                C = B;
            elseif isa(A, 'pbaffine') && isa(B, 'pbaffine')
                if numel(A.degrees) > numel(B.degrees)
                    B.degrees = ...
                        [B.degrees, zeros(1, numel(A.degrees) - numel(B.degrees))];
                elseif numel(B.degrees) > numel(A.degrees)
                    A.degrees = ...
                        [A.degrees, zeros(1, numel(B.degrees) - numel(A.degrees))];
                end
                degrees = A.degrees + B.degrees;
                C = pbpoly(degrees);
            else
                C = times(B, A);
            end
        end
    end
end

