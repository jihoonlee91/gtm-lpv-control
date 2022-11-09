classdef pbinterp < pbasis
    %PBINTERP Interpolation basis function
    %   This basis function is parametrized by the "Gridpoints" and
    %   the corresponding value matrix, "V". 
    %   Syntax:
    %
    %       pb = pbinterp({rho1, rho2, ..., rhon}, V, method)
    %
    %   rho1, rho2, rho...: numeric vectors of grid points for each element 
    %            of the (extended) scheduling signal. (1-by-N1, 1-by-N2,
    %            1-by-N...)
    %   V: value of point at grid, (N1-by-N2-by-N...) 
    %   method (optional): character vector denoting the method used to interpolate between grid points. All methods supported by the built-in function INTERPN are supported. Default: 'linear'.
    %
    %
    %   For example:
    %
    %       Gridpoints = {[0, 1, 4  5], [ 0 2] }
    %       V          = [0, 0, 1, 0; 0 0 0 0]
    %       
    %       pb = pbinterp(Gridpoints,V)
    %
    %   corresponds to the default linear method, 
    %   resulting in a tent function on the first extended scheduling signal
    %   between the values 1 and 5 with a peak at 4 and the second between
    %   0 and 2 with the peak at 0.
    %
    
    properties
        Gridpoints
        V
        Method
    end
    
    methods
        function obj = pbinterp(Gridpoints, V, method)
            %PBINTERP Construct an instance of this class
            %TODO: add additional input parsing
            assert(iscellof(Gridpoints, 'numeric'), 'Gridpoints should be in numeric arrays in cells')
            
             obj.Gridpoints = Gridpoints;
             obj.V = V;
             if nargin == 3
                 obj.Method = method;
             else
                 obj.Method = 'linear';
             end
             obj.sortKey = obj.sortKey_();
             obj.dependenceType = 'interp';
        end
        
        function A = times(A, B)
            if ~strcmp(B.dependenceType, 'no')
                error('LPVcore:pbasis:pbinterp:NoSupport','Multiplication using pbinterp is not supported at this time.')
                %exept for multiplication with the basisfunction bfconst
                %but user doesn't have to know...
            end
        end
        
        function f = feval(obj, rho)
            rho_ = mat2cell(rho,size(rho,1),ones(1,size(rho,2)));
            
            f = interpn(obj.Gridpoints{:}, obj.V, rho_{:}, obj.Method);
        end
        
        function obj = reidx(obj, I)
            %REIDX Reconstructs basis function after transforming rho.
            %   When two pmatrix objects are joined, the ext. scheduling
            %   parameter rho will be augmented with the union of the
            %   rho vectors of the input pmatrix objects. This change
            %   should be propagated to the basis fucntions. REIDX takes
            %   care of reindexing the expression.
            %
            %   Syntax:
            %       p = p.reidx(I)
            %
            %   Inputs:
            %       I (column vector): vector with the samen number of
            %       elements as the old ext. scheduling vector rho. Each
            %       element of I is the corresponding index of the new
            %       ext. scheduling vector.
            %
            %   Outputs:
            %       p: modified basis function
            %
           assert (numel(I) == numel(obj.Gridpoints))
            if numel(I) > 1
                Gridpoints_ = obj.Gridpoints;
                for i = 1:numel(I)
                    Gridpoints_(i) = obj.Gridpoints(I(i)) ;
                end
                obj.Gridpoints = Gridpoints_;

                obj.V = permute(obj.V, I);
                obj.sortKey = obj.sortKey_();         
            end
        end
        
        function s = str(obj, ~)
            % TODO: improve formatting
            s = ['', obj.Method, ' interp.'];
        end
    end
    
    methods (Hidden, Access=protected)
        function diff_(~, ~)
            error('LPVcore:pbasis:pbinterp:NoSupport', 'diff is not supported for interpolation basis functions');
        end

        function n = minNRho_(obj, varargin)
            n = numel(obj.Gridpoints);
        end

        function s = sortKey_(obj)
            s1 = mat2str(cell2mat(obj.Gridpoints));
            if numel(size(obj.V)) > 2
                V_ = reshape(obj.V, size(obj.V,1), []);
                s2 = [mat2str(V_) , 'sizeV = ' , mat2str(size(obj.V))];
            else
                s2 = mat2str(obj.V);
            end
            s3 = obj.Method; 
            s = [s1, s2, s3];
        end
    end
end

