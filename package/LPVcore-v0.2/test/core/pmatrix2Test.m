classdef pmatrix2Test < matlab.unittest.TestCase
    properties (MethodSetupParameter)
        % These properties parametrize pmatrix objects in terms of size,
        % parameter-dependence, time domain, dimension of the scheduling
        % signal and time-map.
        matSize  = struct('empty',[0 0], 'scalar', [1 1],...
              'square', [4 4], 'nonsquare', [4 6], ... 
              'rowvec', [1 4], 'colvec', [4 1])
         %TODO: figure out why size of mat 2 is double the size in 2nd dim
         %as to what is given here, (4x8, 4x12, 1x8, 4x2)
        matSize2 = struct('empty',[0 0], 'scalar', [1 1],...
              'square', [4 4], 'nonsquare', [4 6], ... 
              'rowvec', [1 4], 'colvec', [4 1])
        
        bfunc  = { 'poly', 'const', 'affine', 'custom'}
        bfunc2 = {'affine'}
        
        domain  = {'ct', 'dt', 'undefined'}
        domain2 = {'ct'}
        np = {1, 3};
        np2 = {1};
        
        map  = struct( 'dynamic2', [1 2 1000],'static', 0, 'empty', [])
        map2 = struct('static', 0)
        
        seed = {1}
        

        %TODO: paramiterize the feval function in all tests, currently
        %hardcoded as 2.
        %p = {{2}}; 
    end
    
    properties
        pmat1
        pmat2
        mat
    end
    
    properties (TestParameter)
        % Properties for parametrized testing of individual functions
        freezeArgs =  {{}, {1}, {'''P or Rho should be numeric Right??'''},...
                       {'2*ones(1,pmat.timemap.nrho)'}, {'2*ones(1,pmat.Np)'},...
                       {1 , '''basis'''}}; %{[]}
        pevalArgs = {{1, 1}};
        % p/feval handles an empty array inconsistent {[]} for different pmatrices.
        simplifyArgs = {{0}}
        shiftArgs = {{0}, {1}, {3}, {5}}
        sumArgs = struct('empty', {{}}, 'dim1', {{1}},'dim2', {{2}},...
                        'allDim', {{'all'}})
        diagArgs = struct('empty', {{}}, 'k1', {{1}}, 'negk10', {{-10}},...
                        'k2', {{2}}, 'negk3', {{-3}})
        %powerArgs explode fast and get inaccurate
        powerArgs = { {0}, {1}, {2}, {2.5}}
        mpowerArgs = { {0}, {1}, {2}, {2.5}}
        subsrefArgs = { '1:n, 1:m' , 'n:end, m:end', ...
                        'n,:', ':,m'}
        %TODO: include indexes outside of nxm (n+1 for example) when
        %subsAsign is adjusted
        subsasgnArgsn = { '1' , 'n', ':' , '1:end', 'n:end' }
        subsasgnArgsm = { '1' , '1:m','m'  }
        reshapeArgs = { {[1 2]}, {[1 2 3]}, {1,2,3}, {[],4} , {4, []}} 
        
        funchGR2 = { @ctranspose, @transpose, @diag, @uminus} 
        funchGR2nonP = {@isempty, @isscalar, @isvector, @size}
        funchGR3 = { @blkdiag, @kron } 
        funchGR3compatible = { @minus, @times,@plus}
        catDims = { 1, 2};
    end

    methods (TestMethodSetup, ParameterCombination = 'pairwise')
        
        function methodSetup(testCase,seed, ...
                             matSize,bfunc,domain,map,np,...
                             matSize2,bfunc2,domain2,map2,np2)
            
            testCase.pmat1 = genpmat(matSize ,bfunc ,domain ,map ,np ,seed);
            testCase.pmat2 = genpmat(matSize2,bfunc2,domain2,map2,np2,(seed+1));
            testCase.mat = freeze(testCase.pmat2, 2);
        end
    end
    
    methods (Test, TestTags ={'Group1'})
        % These functions take one pmatrix as input and do not have matlab
        % equivalents to compare
        function testFreeze(testCase, freezeArgs)
            %Both function and test are fully reliant on @()feval
            pmat = testCase.pmat1;

            for i = 1: numel(freezeArgs)
                if ischar(freezeArgs{i})
                    freezeArgs{i} = eval(freezeArgs{i});
                end
            end
            switch numel(freezeArgs)
                case 1
                    if isempty(freezeArgs{1}) && ~isconst(pmat)
                        verifyError(testCase, @() freeze(pmat,freezeArgs{:}), '')
                        return
                    end

                    if ~isnumeric(freezeArgs{:}) || ~(size(freezeArgs{:},2) == pmat.Np || size(freezeArgs{:},2) == 1)
                        verifyError(testCase, @() freeze(pmat,freezeArgs{:}), '')
                        return
                    end

                    if numel(freezeArgs{1})==0 && pmat.Np == 0
                        freezeArgs{1} = [];
                    end
                    rho = freeze(pmat.timemap, freezeArgs{:});
                    GT = feval(pmat, rho);
                    verifyEqual(testCase, freeze(pmat, freezeArgs{:}), GT)
                    
                case 0
                    verifyError(testCase,@() freeze(pmat,freezeArgs{:}), 'MATLAB:minrhs')
                otherwise
                    verifyError(testCase,@() freeze(pmat,freezeArgs{:}), 'MATLAB:TooManyInputs')
            end
        end
        function testPeval(testCase, pevalArgs)
            pmat = testCase.pmat1;

            for i = 1: numel(pevalArgs)
                if ischar(pevalArgs{i})
                    pevalArgs{i} = eval(pevalArgs{i});
                end
            end
            switch numel(pevalArgs)
                case 1
                    if ~isnumeric(pevalArgs{:})
                        %should error here
                        verifyError(testCase,@() peval(pmat,pevalArgs{:}), 'LPVcore:argNaN')
                        return
                    end
                    if isempty(pevalArgs{:}) && size(pevalArgs{:},1) ~= size(pevalArgs{:},2)
                        % inconsistend handling of empty argument,
                        % therefore addition of size(dim1) ~=size(dim2)
                        verifyError(testCase,@() peval(pmat,pevalArgs{:}), 'MATLAB:minrhs')
                        return
                    end
                    if strcmp(pmat.timemap.Domain, 'ct') && size(pevalArgs{:},2) ~= 1
                        %somehow errors here. TODO: find cause
                        %3 different errors
                        %verifyError(testCase,@() peval(pmat,freezeArgs{:}), 'MATLAB:minrhs')
                        %verifyError(testCase,@() peval(pmat,freezeArgs{:}), 'MATLAB:subsassigndimmismatch')
                        %verifyError(testCase,@() peval(pmat,freezeArgs{:}), 'MATLAB:nargoutchk:tooManyOutputs')
                        return
                    end
                    if strcmp(pmat.timemap.Domain, 'ct') && (isempty(pmat.timemap.Map) || size(pevalArgs{:},2) ~= pmat.Np ) 
                        %somehow errors here. TODO: find cause
                        %but not always and 3 different errors
                        %verifyError(testCase,@() peval(pmat,freezeArgs{:}), 'MATLAB:minrhs')
                        %verifyError(testCase,@() peval(pmat,freezeArgs{:}), 'MATLAB:subsassigndimmismatch')
                        %verifyError(testCase,@() peval(pmat,freezeArgs{:}), 'MATLAB:nargoutchk:tooManyOutputs')
                        return
                    end
                    if size(pevalArgs{:},2) ~= pmat.Np && size(pevalArgs{:},2) ~= 1 && ~isconst(pmat)
                        %does wierd stuff with sizes here as well
                        if size(pevalArgs{:},2) > pmat.timemap.nrho
                            verifyError(testCase,@() peval(pmat,pevalArgs{:}), 'MATLAB:subsassigndimmismatch')
                            return
                        elseif  size(pevalArgs{:},2) < pmat.Np
                            verifyError(testCase,@() peval(pmat,pevalArgs{:}), 'MATLAB:minrhs')
                            return
                        end
                    end
                    
                    rho = eval(pmat.timemap, pevalArgs{:});
                    if isempty(rho) && size(rho, 1) == 1
                        GT = freeze(pmat);
                    else
                        GT = feval(pmat, rho);
                    end
                    verifyEqual(testCase, peval(pmat, pevalArgs{:}), GT, 'reltol', 1E-9)
                    
                case 0
                    verifyError(testCase,@() peval(pmat,pevalArgs{:}), 'MATLAB:minrhs')
                otherwise
                    Ts = 1;
                    verifyError(testCase,@() peval(pmat,Ts,pevalArgs{:}), 'MATLAB:TooManyInputs')
            end
        end
        function testFeval(testCase, freezeArgs)
            pmat = testCase.pmat1;
            
            for i = 1: numel(freezeArgs)
                if ischar(freezeArgs{i})
                    freezeArgs{i} = eval(freezeArgs{i});
                end
            end
            switch numel(freezeArgs)
                case 0
                	verifyError(testCase,@() feval(pmat,freezeArgs{:}), 'MATLAB:minrhs')
                    
                case 1
                    if ~isnumeric(freezeArgs{1}) 
                        verifyError(testCase,@() feval(pmat,freezeArgs{:}), 'LPVcore:argNaN')
                        return
                    end
                    if isempty(pmat)
                        %quite some 'inconsistent' errors here
                        %not all empty error though
                        % mainly on the combintation of customs and
                        % incorrect size of freezeArgs (remove
                        % parameterCombination = pairwise)
                        return
                    end
                    N = size(freezeArgs{:}, 1);
                    psi = zeros(N, pmat.numbfuncs);
                    if size(freezeArgs{1}, 2) == 1
                        freezeArgs{1} = freezeArgs{1} * ones(1, pmat.timemap.nrho);
                    end
                    if size(freezeArgs{:},2) < pmat.timemap.nrho && ~isconst(pmat)
                        %if the index exceeds either the index in a custom
                        %or poly this errors.
                        %uncertain of errors in case pmat contains both 
                        if iscustom(pmat) || ispoly(pmat)
                            if iscustom(pmat)
                                %some empty(pmat) error here
                                verifyError(testCase,@() feval(pmat,freezeArgs{:}), 'LPVcore:pmat:argNumel')
                            end
                            if ispoly(pmat)
                                verifyError(testCase,@() feval(pmat,freezeArgs{:}), 'LPVcore:pmat:argNumel')
                            end
                        else
                           %should never happen *it does with empty*
                           verifyEqual(testCase,1,0)
                        end
                        
                        return
                    end
                    for i=1:pmat.numbfuncs
                        psi(:, i) = eval(pmat.bfuncs{i}, freezeArgs{1});
                    end
                    [n, m, o] = size(pmat.matrices);
                    C = reshape(pmat.matrices, m,  o * n);
                    PSI = kron(psi', eye(n));
                    GT = reshape(C * PSI, n, m, N);
                    verifyEqual(testCase, feval(pmat, freezeArgs{:}), GT, 'reltol', 1E-9)
                case 2 
                    if ~isnumeric(freezeArgs{1})
                        verifyError(testCase,@() feval(pmat,freezeArgs{:}), '')
                        return
                    end
                    N = size(freezeArgs{1}, 1);
                    psi = zeros(N, pmat.numbfuncs);
                    if isscalar(freezeArgs{1})
                        freezeArgs{1} = ones(1, pmat.timemap.nrho) * freezeArgs{1};
                    end
                    for i=1:pmat.numbfuncs
                        psi(:, i) = eval(pmat.bfuncs{i}, freezeArgs{1});
                    end
                    if strcmp(freezeArgs{2}, 'basis')
                        GT = psi;
                        verifyEqual(testCase, feval(pmat, freezeArgs{:}), GT, 'reltol', 1E-9)
                        return
                    else
                        [n, m, o] = size(pmat.matrices);
                        C = reshape(pmat.matrices, m,  o * n);
                        PSI = kron(psi', eye(n));
                        GT = reshape(C * PSI, n, m, N);
                        verifyEqual(testCase, feval(pmat, freezeArgs{:}), GT, 'reltol', 1E-9)
                    end
                otherwise
            end
        end
        function testShift(testCase, shiftArgs)%testPshift, testPdiff
            pmat = testCase.pmat1;
            tm = pmat.timemap;
            tm.Map = tm.Map + shiftArgs{:};
            GT = pmatrix(pmat.matrices, pmat.bfuncs, 'SchedulingTimeMap', tm);
            
            if strcmp(pmat.timemap.Domain,'undefined')
                % verifyEqual(testCase, pdiff(pmat,shiftArgs{:}), GT, 'absTol', 1E-12)
                verifyEqual(testCase, pshift(pmat,shiftArgs{:}), GT, 'absTol', 1E-12)
            elseif strcmp(pmat.timemap.Domain,'ct') && shiftArgs{:} >= 0%&&  floor(shiftArgs{:}) == shiftArgs{:})
                % verifyEqual(testCase, pdiff(pmat,shiftArgs{:}), GT, 'absTol', 1E-12)
                verifyError(testCase,@() pshift(pmat,shiftArgs{:}),'LPVcore:pmat:wrongTimemap')
            elseif strcmp(pmat.timemap.Domain,'dt') %&&  floor(shiftArgs{:}) == shiftArgs{:})
                verifyError(testCase,@() pdiff(pmat,shiftArgs{:}), 'LPVcore:pmat:wrongTimemap')
                % verifyEqual(testCase, pshift(pmat,shiftArgs{:}), GT, 'absTol', 1E-12)
            else
                % verifyError(testCase,@() pdiff(pmat,shiftArgs{:}), '')
                verifyError(testCase,@() pshift(pmat,shiftArgs{:}), 'LPVcore:pmat:wrongTimemap')
            end
           
        end

%         function testSimplify(testCase, simplifyArgs) %TODO:write it
%             1
%         end
        
    end
    
    methods (Test , TestTags ={'Group2'})
        %This group of functions takes 1 pmatrix as input and can be
        %checked by feval or a short/simple if statement.
        function testGroup2basis(testCase, funchGR2)
            pmat = testCase.pmat1;
            p = 2;
            funch = funchGR2;
            verifyEqual(testCase, feval(funch(pmat), p), funch(feval(pmat, p)), 'relTol', 1E-9);
        end
        function testGroup2nonPout(testCase, funchGR2nonP)
            pmat = testCase.pmat1;
            p = 2;
            funch = funchGR2nonP;
            verifyEqual(testCase, funch(pmat), funch(feval(pmat, p)), 'relTol', 1E-9);
        end
        function testSum(testCase,sumArgs)
            pmat = testCase.pmat1;
            p = 2;
            verifyEqual(testCase, sum(feval(pmat, p), sumArgs{:}), feval(sum(pmat, sumArgs{:}), p), 'relTol', 1E-9)
        end

        function testIsaffine(testCase)
            %TODO: test with affine pbcustom functions 
            pmat = testCase.pmat1;
            Tpmat = simplify(pmat);
            c = true;
            for i = 1: numel(Tpmat.bfuncs)
                if ~isa(Tpmat.bfuncs{i},'pbaffine')
                    c = false;
                end
            end
            if isempty(pmat); c = true; end
            verifyEqual(testCase, isaffine(pmat), c,'AbsTol', 1E-10);

        end
        function testIsconst(testCase)
            %TODO: test with constant pbcustom functions
            pmat = testCase.pmat1;
            Tpmat = simplify(pmat);
            c = true;
            for i = 1: numel(Tpmat.bfuncs)
                if ~isa(Tpmat.bfuncs{i},'pbconst')
                    c = false;
                end
            end
            if isempty(Tpmat)
                c = true;
            end
            verifyEqual(testCase, isconst(pmat), c,'AbsTol', 1E-10);
        end
        function testIsempty(testCase)
            pmat = testCase.pmat1;
            p = 2;
            c = isempty(freeze(pmat, p));
            verifyEqual(testCase, isempty(pmat),c,'AbsTol', 1E-10);
        end
        function testIspoly(testCase)
            pmat = testCase.pmat1;
            Tpmat = simplify(pmat);
            c = true;
            if ~isempty(pmat)
                for i = 1:numel(Tpmat.bfuncs)
                    if ~isa(Tpmat.bfuncs{i},'pbpoly') && max(abs(Tpmat.matrices(:, :, i)), [],'all') > eps 
                        c = false;
                    end
                end
            end
            verifyEqual(testCase, ispoly(pmat), c,'AbsTol', 1E-10);
        end
        function testIscustom(testCase)
            pmat = testCase.pmat1;
            Tpmat = simplify(pmat);
            c = false;
            if ~isempty(pmat)
                for i = 1:numel(Tpmat.bfuncs)
                    if isa(Tpmat.bfuncs{i},'pbcustom') && max(abs(Tpmat.matrices(:, :, i)), [],'all') > eps 
                        c = true;
                    end
                end
            end
            verifyEqual(testCase, iscustom(pmat), c,'AbsTol', 1E-10);
        end
        function testIssquare(testCase)
            pmat = testCase.pmat1;
            p = 2*ones(1,pmat.timemap.nrho);
            issq = @(mat) (size(mat,1) == size(mat,2)); 
            c = issq(feval(pmat, p));
            verifyEqual(testCase, issquare(pmat),c,'AbsTol', 1E-10);
        end
        function testPower(testCase,powerArgs) 
            %TODO: when functionality is added, add matrix/array inputs and move to group 2.5 or 3
            %TODO: make this function more accurate
            pmat = testCase.pmat1;
            p = 1.5;
            if powerArgs{:} == floor(powerArgs{:})
                verifyEqual(testCase,feval(pmat, p).^powerArgs{:}, feval(pmat.^powerArgs{:}, p), 'reltol',  1E-2);
                if strcmp(powerArgs, "0")
                        verifyEqual(testCase, pmat.^0, pmatrix(ones(size(pmat)), {pbconst}, 'SchedulingTimeMap', A.timemap))
                end
            else
                verifyError(testCase,@() pmat.^powerArgs{:}, 'LPVcore:pmat:argPosIntOnly')
            end
        end
        function testMpower(testCase,mpowerArgs) 
            %TODO: when functionality is added, add matrix/array inputs and move to group 2.5 or 3
            pmat = testCase.pmat1;
            p = 1.25;
            if issquare(pmat) && mpowerArgs{:} == floor(mpowerArgs{:})
                verifyEqual(testCase,feval(pmat, p)^mpowerArgs{:}, feval(pmat^mpowerArgs{:}, p), 'reltol',  1E-9);
                if strcmp(mpowerArgs, "0")
                    verifyEqual(testCase, pmat^0, pmatrix(eye(size(pmat, 1)), {pbconst}, 'SchedulingTimeMap', pmat.timemap))
                end
            elseif mpowerArgs{:} == floor(mpowerArgs{:})
                verifyError(testCase,@() pmat^mpowerArgs{:}, 'LPVcore:pmat:squareOnly')
            else
                verifyError(testCase,@() pmat^mpowerArgs{:}, 'LPVcore:pmat:argPosIntOnly')%this is tested first in function mpower()
            end
        end
        function testSubsref(testCase,subsrefArgs) 
            %TODO:simplify test for large NxM
            pmat = testCase.pmat1;
            p = 2;
            mat2 = feval(pmat,p); %#ok<NASGU>
            for n = 1: size(pmat,1)
                for m = 1 : size(pmat,2)
                    tpmat = eval(['pmat(', subsrefArgs , ');']);
                    tmat = eval(['mat2(', subsrefArgs , ');']);

                    verifyEqual(testCase, feval(tpmat, p), tmat, 'relTol', 1E-9);
                end
            end 
        end
        function testSubsasgn(testCase, subsasgnArgsn, subsasgnArgsm) 
            pmat = testCase.pmat1;
            p = 2;
            mat2 = feval(pmat,p);
            [n, m] = size(mat2);


            if strcmp(':',subsasgnArgsn)
                Nn = n;
            else
                Nn = eval(['size( ', strrep(subsasgnArgsn,'end','n') , ' ,2)']);
            end
            
            if strcmp(':',subsasgnArgsm)
                Nm = m;
            else
                Nm = eval(['size( ', strrep(subsasgnArgsm,'end','m') , ' ,2)']);
            end

            if any([n, m] == [0 0])
                verifyError(testCase, @() mat2(n,m), 'MATLAB:badsubscript')
            else
                eval(['mat2(', subsasgnArgsn ,' ,',  subsasgnArgsm , ') = ones(', num2str(Nn) ,',', num2str(Nm) ,');'])
                eval(['pmat(', subsasgnArgsn ,' ,',  subsasgnArgsm , ') = ones(', num2str(Nn) ,',', num2str(Nm) ,');'])
                verifyEqual(testCase, feval(pmat, p) , mat2 , 'reltol', 1E-10);
            end
        end
        function testVectorization(testCase)
            pmat = testCase.pmat1;
            p = 2;
            f = feval(pmat, p);
            verifyEqual(testCase, feval(pmat(:), p), f(:), 'AbsTol', 1E-10);
            verifyEqual(testCase, feval(pmat(:)', p), f(:)', 'AbsTol', 1E-10);
        end
        function testReshape(testCase, reshapeArgs)
            pmat = testCase.pmat1;
            p = 2;
            if numel(reshapeArgs) > 2 || numel(reshapeArgs{1}) > 2
                verifyError(testCase,@() reshape(pmat, reshapeArgs{:}), 'LPVcore:DimConvention')
                return
            end
            
            if prod([reshapeArgs{:}]) ~= prod(size(pmat))  %#ok<PSIZE> 
                %TODO: replace with numel when function exists
                if numel(reshapeArgs) == 1 
                    verifyError(testCase,@() reshape(pmat, reshapeArgs{:}), 'MATLAB:getReshapeDims:notSameNumel')
                    return
                else
                    if ~(isempty(reshapeArgs{1}) || isempty(reshapeArgs{2}))
                        verifyError(testCase,@() reshape(pmat, reshapeArgs{:}), 'MATLAB:getReshapeDims:notSameNumel')
                        return
                    else
                       if ~(rem(prod(size(pmat)),prod([reshapeArgs{:}])) == 0 ) %#ok<PSIZE> 
                           verifyError(testCase,@() reshape(pmat, reshapeArgs{:}), 'MATLAB:getReshapeDims:notDivisible')
                           return
                       end
                    end
                end
            end
            verifyEqual(testCase, reshape(feval(pmat,p),reshapeArgs{:}), feval(reshape(pmat, reshapeArgs{:}),p), 'reltol', 1E-9)
        end

    end
        
    methods (Test , TestTags ={'Group2.5'})
        % This group of functions takes a pmatrix and a constant matrix as
        % input and can be checked with feval and and a simple ifstatement
        function testldivide(testCase)
            p_mat1 = testCase.pmat1;
            mat2 = testCase.mat;
            p = 2;
            
            if all(compatibleSize(p_mat1,mat2),'all' )
                verifyEqual(testCase, feval((mat2.\p_mat1), p),mat2.\ feval(p_mat1, p), 'relTol', 1E-12);
            else
                % The error thrown was changed in R2021a
                if verLessThan('MATLAB', '9.10')
                    verifyError(testCase, @() mat2.\p_mat1, 'MATLAB:dimagree')
                else
                    verifyError(testCase, @() mat2.\p_mat1, 'MATLAB:sizeDimensionsMustMatch');
                end
            end 
        end
        function testmldivide(testCase)
            p_mat1 = testCase.pmat1;
            mat2 = testCase.mat;
            p = 2;
            
            if size(p_mat1,1) == size(mat2,1) || isscalar(mat2)
                verifyEqual(testCase, feval((mat2\p_mat1), p),mat2\ feval(p_mat1, p), 'relTol', 1E-12);
            else
                verifyError(testCase, @() mat2\p_mat1, 'MATLAB:dimagree')
            end 
        end
        function testrdivide(testCase)
            p_mat1 = testCase.pmat1;
            mat2 = testCase.mat;
            p = 2;
            
            if all(compatibleSize(p_mat1,mat2),'all' )
                verifyEqual(testCase, feval((p_mat1./mat2), p),feval(p_mat1, p)./mat2, 'relTol', 1E-12);
            else
                % The error thrown was changed in R2021a
                if verLessThan('MATLAB', '9.10')
                    verifyError(testCase, @() p_mat1./mat2, 'MATLAB:dimagree')
                else
                    verifyError(testCase, @() p_mat1 ./mat2, 'MATLAB:sizeDimensionsMustMatch');
                end
            end  
        end
        function testmrdivide(testCase)
            p_mat1 = testCase.pmat1;
            mat2 = testCase.mat;
            p = 2;
            
            if size(p_mat1,2) == size(mat2,2) || isscalar(mat2)
                verifyEqual(testCase, feval((p_mat1/mat2), p),feval(p_mat1, p)/mat2, 'relTol', 1E-12);
            else
                verifyError(testCase, @() p_mat1/mat2, 'MATLAB:dimagree')
            end  
        end
    end
    
    methods (Test , TestTags ={'Group3'})
        % This group of functions takes 2 pmatrices as input and can be
        % checked by feval and a simple ifstatement.
        function testGroup3basis(testCase, funchGR3)
            p_mat1 = testCase.pmat1;
            p_mat2 = testCase.pmat2;
            funch = funchGR3;
            
            basisgroup3test(testCase,p_mat1,p_mat2, funch)
        end
        function testGroup3compatSize(testCase, funchGR3compatible)
            p_mat1 = testCase.pmat1;
            p_mat2 = testCase.pmat2;
            funch = funchGR3compatible;
            
            if all(compatibleSize(p_mat1,p_mat2),'all')
            basisgroup3test(testCase,p_mat1,p_mat2, funch)
            else
                if ~timemap.unionDomain(p_mat1.timemap.Domain, p_mat2.timemap.Domain)
                    verifyError(testCase, @() funch(p_mat1,p_mat2), '')
                else
                verifyError(testCase,@() funch(p_mat1,p_mat2), ?MException );
                %3 different exeptions are thown, 'MATLAB:sizeDimensionsMustMatch',
                %'MATLAB:dimagree' & 'LPVcore:pmat:dimAgree', TODO: make sure
                %its just these 3 exeptions it catches, not all.
                end
            end
        end
        function testCat(testCase, catDims)
            p_mat1 = testCase.pmat1;
            p_mat2 = testCase.pmat2;
            funch = @(A,B) cat(catDims,A,B);
            
            if size(p_mat1,3-catDims) == size(p_mat2,3-catDims) || any(size(p_mat1) == [0 0],'all') || any(size(p_mat2) == [0 0],'all')
                basisgroup3test(testCase,p_mat1,p_mat2, funch)
            else
                if ~timemap.unionDomain(p_mat1.timemap.Domain, p_mat2.timemap.Domain)
                    verifyError(testCase, @() funch(p_mat1,p_mat2), '');
                else
                    verifyError(testCase,@() funch(p_mat1,p_mat2), 'MATLAB:catenate:dimensionMismatch')
                end
            end
        end
        function testHorzcat(testCase)
            p_mat1 = testCase.pmat1;
            p_mat2 = testCase.pmat2;
            funch = @horzcat;
            
            if size(p_mat1,1) == size(p_mat2,1) || any(size(p_mat1) == [0 0],'all') || any(size(p_mat2) == [0 0],'all')
                basisgroup3test(testCase,p_mat1,p_mat2, funch)
            else
                if ~timemap.unionDomain(p_mat1.timemap.Domain, p_mat2.timemap.Domain)
                    verifyError(testCase, @() funch(p_mat1,p_mat2), '');
                else
                    verifyError(testCase,@() funch(p_mat1,p_mat2), 'MATLAB:catenate:dimensionMismatch')
                end
            end
        end
        function testVertcat(testCase)
            p_mat1 = testCase.pmat1;
            p_mat2 = testCase.pmat2;
            funch = @vertcat;
            
            if size(p_mat1,2) == size(p_mat2,2) || any(size(p_mat1) == [0 0],'all') || any(size(p_mat2) == [0 0],'all')
                basisgroup3test(testCase,p_mat1,p_mat2, funch)
            else
                if ~timemap.unionDomain(p_mat1.timemap.Domain, p_mat2.timemap.Domain)
                    verifyError(testCase, @() funch(p_mat1,p_mat2), '');
                else
                    verifyError(testCase,@() funch(p_mat1,p_mat2), 'MATLAB:catenate:dimensionMismatch')
                end
            end
        end
    end
    
end

%% Local functions
function basisgroup3test(testCase,p_mat1,p_mat2, funch)
    p = 2;
    if ~timemap.unionDomain(p_mat1.timemap.Domain, p_mat2.timemap.Domain)
        verifyError(testCase, @() funch(p_mat1,p_mat2), '');
    else
        verifyEqual(testCase, feval(funch(p_mat1, p_mat2), p),funch(feval(p_mat1, p), feval(p_mat2, p)), 'relTol', 1E-12);    
    end 
end

function comp = compatibleSize(obj1, obj2)
    a = numel(size(obj1));
    b = numel(size(obj2));
    if a < b; a = b; end
    comp = zeros(1,a);
    for i = 1: a
        if (size(obj1,i) == size(obj2,i) || size(obj1,i) == 1 || size(obj2,i) == 1)
            comp(i) = true;
        end
    end
end
function pmat = genpmat(matSize,bfunc,domain,map,np, seed)
    if nargin == 5
        rng(seed);
    else
        rng(0);
    end

    tm = genMap(domain, map, np);

    nrho = size(map,2) * np;

    switch bfunc
        case 'const'
            mat = rand(matSize)+1i*rand(matSize);
                mat = mat/sqrt(2);
            pmat = pmatrix(mat,...
                       { pbconst },...
                       'SchedulingTimeMap', tm);
        case 'affine'
            mat = rand([matSize, nrho+1])+1i*rand([matSize, nrho+1]);
            pmat = pmatrix(mat,... %and constant, if purely define go for mat(:,:,1)
                       'BasisType',  bfunc,...
                       'SchedulingTimeMap', tm);
        case 'poly'
            mat = rand([matSize, nrho])+ 1i*rand([matSize, nrho]);

            param = cell(1, nrho);
            for i = 1: nrho
                param{i} = randi(5,1,nrho);
            end
            pmat = pmatrix(mat,...
                       'BasisType',  bfunc,...
                       'BasisParametrization', param,...
                       'SchedulingTimeMap', tm);
        case 'custom'
            mat = rand([matSize, nrho])+ 1i*rand([matSize, nrho]);
            if nrho == 0; pbcust = {};else; pbcust{nrho} = []; end
            for i=1:nrho
                pbcust{i} = str2func(['@(rho) sin(rho(:, ' num2str(i) ' ))']); %#ok<AGROW>
            end
            
            pmat = pmatrix(mat,...
                       'BasisType',  bfunc,...               
                       'BasisParametrization', pbcust,...
                       'SchedulingTimeMap', tm);
    end
end

function map = genMap(domain, map, np)
    map = timemap(map,...
                domain, ...
                'dimension', np);
end