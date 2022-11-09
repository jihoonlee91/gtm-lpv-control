classdef pbconst < pbaffine
    %PBCONST Parameter-independent basis function
    %   This basis function represents the constant 1.
    
    methods
        function obj = pbconst
            obj = obj@pbaffine(0);
            obj.dependenceType = 'no';
        end
        
        function s = str(~, ~)
            s = '1';
        end
        
        function C = times(A, B)
            if isa(A, 'pbconst') && isa(B, 'pbconst')
                C = A;
            else
                C = times(B, A);
            end
        end
        
        function obj = reidx(obj, ~)
            % Reindexing a constant has no effect.
        end
        
        function f = eval(~, ~)
            f = 1;
        end
    end
end

