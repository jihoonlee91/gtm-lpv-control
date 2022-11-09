function tests = gridvarTest
%GRIDVARTEST Test gridvar class
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
%%
rng(1)

% gridvar object
A = randn(5,8,4,5)+randn(5,8,4,5)*1i;
Ag = gridvar(A);

B = randn(8,5)+randn(8,5)*1i;
Bg = gridvar(B);

C = sdpvar(3,3,3,'sym');
Cg = gridvar(C);

C2 = sdpvar(3,3,3,'sym');
C2g = gridvar(C2);

D = cat(3,-eye(3),-eye(3)*1.1,-eye(3)*1.2);
Dg = gridvar(D);

% Add to testCase
testCase.TestData.A = A;
testCase.TestData.B = B;
testCase.TestData.C = C;
testCase.TestData.C2 = C2;
testCase.TestData.D = D;
testCase.TestData.Ag = Ag;
testCase.TestData.Bg = Bg;
testCase.TestData.Cg = Cg;
testCase.TestData.C2g = C2g;
testCase.TestData.Dg = Dg;
end

function testSize(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    rng(1);

    % Multiple gridpoints
    [nA,mA,nGA] = size(A);

    n = size(Ag,1);
    check(1) = n == nA;

    m = size(Ag,2);
    check(2) = m == mA;

    nG = size(Ag,3);
    check(3) = (nG == nGA) && (nG == Ag.Ng);

    [n,m,nG] = size(Ag);
    check(4) = (n == size(A,1)) && (m == size(A,2)) && (nG == Ag.Ng);

    % One gridpoint
    n = size(Bg,1);
    check(5) = n == size(B,1);

    m = size(Bg,2);
    check(6) = m == size(B,2);

    nG = size(Bg,3);
    check(7) = (nG == size(B,3)) && (nG == Bg.Ng);

    [n,m,nG] = size(Bg);
    check(8) = (n == size(B,1)) && (m == size(B,2)) && (nG == Bg.Ng);

    verifyTrue(testCase, all(check));
end

function testSubsref(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    rng(1);

    % Multiple gridpoints
    ind1 = randi(size(Ag,1),1,2);
    ind2 = randi(size(Ag,2),1,2);

    C1 = Ag(ind1(1):ind1(2),ind2(1):ind2(2));

    ind3 = randi(Ag.Ng);

    check(1) = norm(C1.matrices{ind3}-A(ind1(1):ind1(2),ind2(1):ind2(2),ind3)) == 0;

    % Multiple gridpoints (end)
    ind1 = randi(size(Ag,1),1,2);
    ind2 = randi(size(Ag,2),1,2);

    C2 = Ag(ind1(1):end,ind2(1):end);

    ind3 = randi(Ag.Ng);

    check(2) = norm(C2.matrices{ind3}-A(ind1(1):end,ind2(1):end,ind3)) == 0;

    % Multiple gridpoints (:)
    % row
    ind1 = randi(size(Ag,1));

    C3 = Ag(ind1,:);

    ind3 = randi(Ag.Ng);

    check(3) = norm(C3.matrices{ind3}-A(ind1,:,ind3)) == 0;

    % column
    ind1 = randi(size(Ag,2));

    C4 = Ag(:,ind1);

    ind3 = randi(Ag.Ng);

    check(4) = norm(C4.matrices{ind3}-A(:,ind1,ind3)) == 0;

    % vectorize
    C5 = Ag(:);

    ind3 = randi(Ag.Ng);
    
    Am = A(:,:,ind3);
    check(5) = norm(C5.matrices{ind3}-Am(:)) == 0;

    verifyTrue(testCase, all(check));

    % One gridpoint
    ind1 = randi(size(Bg,1),1,2);
    ind2 = randi(size(Bg,2),1,2);

    C6 = Bg(ind1(1):ind1(2),ind2(1):ind2(2));

    check(6) = norm(C6.matrices{1}-B(ind1(1):ind1(2),ind2(1):ind2(2))) == 0;

    % Multiple gridpoints (end)
    ind1 = randi(size(Bg,1),1,2);
    ind2 = randi(size(Bg,2),1,2);

    C7 = Bg(ind1(1):end,ind2(1):end);

    check(7) = norm(C7.matrices{1}-B(ind1(1):end,ind2(1):end)) == 0;

    % Multiple gridpoints (:)
    % row
    ind1 = randi(size(Bg,1));

    C8 = Bg(ind1,:);

    check(8) = norm(C8.matrices{1}-B(ind1,:)) == 0;

    % column
    ind1 = randi(size(Bg,2));

    C9 = Bg(:,ind1);

    check(9) = norm(C9.matrices{1}-B(:,ind1)) == 0;

    % vectorize
    C10 = Bg(:);
    
    check(10) = norm(C10.matrices{1}-B(:)) == 0;

    verifyTrue(testCase, all(check));
end

function testSubasgn(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    rng(1);

    % Multiple gridpoints
    ind1 = randi(size(Ag,1),1,2);
    ind2 = randi(size(Ag,2),1,2);

    mat = randn(max(ind1)-min(ind1)+1,max(ind2)-min(ind2)+1);

    C1 = Ag;
    C1(ind1(1):ind1(2),ind2(1):ind2(2)) = mat;

    ind3 = randi(Ag.Ng);
    Ac = A;
    Ac(ind1(1):ind1(2),ind2(1):ind2(2),ind3) = mat;

    check(1) = norm(C1.matrices{ind3}-Ac(:,:,ind3)) == 0;

    % row
    ind1 = randi(size(Ag,1));

    mat = randn(1,size(Ag,2));

    C2 = Ag;
    C2(ind1,:) = mat;

    ind3 = randi(Ag.Ng);
    Ac = A;
    Ac(ind1,:,ind3) = mat;

    check(2) = norm(C2.matrices{ind3}-Ac(:,:,ind3)) == 0;

    % column
    ind1 = randi(size(Ag,1));

    mat = randn(size(Ag,1),1);

    C3 = Ag;
    C3(:,ind1) = mat;

    ind3 = randi(Ag.Ng);
    Ac = A;
    Ac(:,ind1,ind3) = mat;

    check(3) = norm(C3.matrices{ind3}-Ac(:,:,ind3)) == 0;

    % One gridpoint
    ind1 = randi(size(Bg,1),1,2);
    ind2 = randi(size(Bg,2),1,2);

    mat = randn(max(ind1)-min(ind1)+1,max(ind2)-min(ind2)+1);

    C4 = Bg;
    C4(ind1(1):ind1(2),ind2(1):ind2(2)) = mat;

    Bc = B;
    Bc(ind1(1):ind1(2),ind2(1):ind2(2)) = mat;

    check(4) = norm(C4.matrices{1}-Bc) == 0;

    % row
    ind1 = randi(size(Bg,1));

    mat = randn(1,size(Bg,2));

    C5 = Bg;
    C5(ind1,:) = mat;

    Bc = B;
    Bc(ind1,:) = mat;

    check(5) = norm(C5.matrices{1}-Bc) == 0;

    % column
    ind1 = randi(size(Bg,1));

    mat = randn(size(Bg,1),1);

    C6 = Bg;
    C6(:,ind1) = mat;

    Bc = B;
    Bc(:,ind1) = mat;

    check(6) = norm(C6.matrices{1}-Bc) == 0;

    verifyTrue(testCase, all(check));
end

function testBlkdiag(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    rng(1);

    % Two gridvar objects with same number of gridpoints
    C1 = blkdiag(Ag,Ag);

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-blkdiag(A(:,:,ind1),A(:,:,ind1))) == 0;

    % Two gridvar objects where one has only one gridpoint
    C2 = blkdiag(Ag,Bg);

    ind2 = randi(Ag.Ng);
    check(2) = norm(C2.matrices{ind2}-blkdiag(A(:,:,ind2),B)) == 0;

    % Two gridvar objects where one has only one gridpoint (swapped
    % arguments)
    C3 = blkdiag(Bg,Ag);

    ind3 = randi(Ag.Ng);
    check(3) = norm(C3.matrices{ind3}-blkdiag(B,A(:,:,ind3))) == 0;

    % Gridvar and contant
    C4 = blkdiag(Ag,B);

    ind4 = randi(Ag.Ng);
    check(4) = norm(C4.matrices{ind4}-blkdiag(A(:,:,ind4),B)) == 0;

    % Constant and gridvar
    C5 = blkdiag(B,Ag);

    ind5 = randi(Ag.Ng);
    check(5) = norm(C5.matrices{ind5}-blkdiag(B,A(:,:,ind5))) == 0;

    % Check
    verifyTrue(testCase, all(check)); 
end

function testDiag(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    rng(1);

    % Gridvar matrix object with > 1 gridpoints
    C1 = diag(Ag);

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-diag(A(:,:,ind1))) == 0;

    % Gridvar with one gridpoint
    C2 = diag(Bg);

    check(2) = norm(C2.matrices{1}-diag(B)) == 0;

    % Gridvar vector with > 1 gridpoints
    C3 = diag(Ag(:));

    ind3 = randi(Ag.Ng);
    check(3) = norm(C3.matrices{ind3}-diag(reshape(A(:,:,ind3),[],1))) == 0;

    % Gridvar vector with one gridpoint
    C4 = diag(Bg(:));

    check(4) = norm(C4.matrices{1}-diag(B(:))) == 0;

    % Check
    verifyTrue(testCase, all(check)); 
end

function testKron(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    rng(1);

    % Two gridvar objects with same number of gridpoints
    C1 = kron(Ag,Ag);

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-kron(A(:,:,ind1),A(:,:,ind1))) == 0;

    % Two gridvar objects where one has only one gridpoint
    C2 = kron(Ag,Bg);

    ind2 = randi(Ag.Ng);
    check(2) = norm(C2.matrices{ind2}-kron(A(:,:,ind2),B)) == 0;

    % Two gridvar objects where one has only one gridpoint (swapped
    % arguments)
    C3 = kron(Bg,Ag);

    ind3 = randi(Ag.Ng);
    check(3) = norm(C3.matrices{ind3}-kron(B,A(:,:,ind3))) == 0;

    % Gridvar and contant
    C4 = kron(Ag,B);

    ind4 = randi(Ag.Ng);
    check(4) = norm(C4.matrices{ind4}-kron(A(:,:,ind4),B)) == 0;

    % Constant and gridvar
    C5 = kron(B,Ag);

    ind5 = randi(Ag.Ng);
    check(5) = norm(C5.matrices{ind5}-kron(B,A(:,:,ind5))) == 0;

    % Check
    verifyTrue(testCase, all(check)); 
end

function testTranspose(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    % Gridvar object with > 1 gridpoints
    C1 = Ag';

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-A(:,:,ind1)') == 0;

    % Gridvar with one gridpoint
    C2 = Bg';

    check(2) = norm(C2.matrices{1}-B') == 0;

    verifyTrue(testCase, all(check));
end

function testInv(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    % Gridvar object with > 1 gridpoints
    n = min(size(Ag));
    C1 = inv(Ag(1:n,1:n));

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-inv(A(1:n,1:n,ind1))) == 0;

    % Gridvar with one gridpoint
    n = min(size(Bg));
    C2 = inv(Bg(1:n,1:n));

    check(2) = norm(C2.matrices{1}-inv(B(1:n,1:n))) == 0;

    verifyTrue(testCase, all(check));
end

function testNumel(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    % Gridvar object with > 1 gridpoints
    C1 = numel(Ag);

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1-numel(A(:,:,ind1))) == 0;

    % Gridvar with one gridpoint
    C2 = numel(Bg);

    check(2) = norm(C2-numel(B)) == 0;

    verifyTrue(testCase, all(check));
end

function testGetMatrices(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    % Gridvar object with > 1 gridpoints
    C1 = getMatrices(Ag);

    check(1) = all(C1 - A == 0, 'all');

    % Gridvar with one gridpoint
    C2 = getMatrices(Bg);

    check(2) = all(C2 - B == 0, 'all');

    verifyTrue(testCase, all(check));
end

function testUminus(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    % Gridvar object with > 1 gridpoints
    C1 = -Ag;

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-(-A(:,:,ind1))) == 0;

    % Gridvar with one gridpoint
    C2 = -Bg;

    check(2) = norm(C2.matrices{1}-(-B)) == 0;

    verifyTrue(testCase, all(check));
end

function testCat(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    rng(1);

    % Gridvar objects with same number of gridpoints
    C1 = [Ag,Ag;Ag,Ag];

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-...
        [A(:,:,ind1),A(:,:,ind1);...
        A(:,:,ind1),A(:,:,ind1)]) == 0;

    % Two gridvar objects where one has only one gridpoint
    C2 = [Ag,Bg';Bg',Ag];

    ind2 = randi(Ag.Ng);
    check(2) = norm(C2.matrices{ind2}-...
        [A(:,:,ind2),B';...
        B',A(:,:,ind2)]) == 0;

    % Gridvar and contant
    C3 = [Ag,B';B',Ag];

    ind3 = randi(Ag.Ng);
    check(3) = norm(C3.matrices{ind3}-...
        [A(:,:,ind3),B';...
        B',A(:,:,ind3)]) == 0;

    % Check
    verifyTrue(testCase, all(check)); 
end

function testMtimes(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    rng(1);

    % Two gridvar objects with same number of gridpoints
    C1 = Ag*Ag';

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-A(:,:,ind1)*A(:,:,ind1)') == 0;

    % Two gridvar objects where one has only one gridpoint
    C2 = Ag*Bg;

    ind2 = randi(Ag.Ng);
    check(2) = norm(C2.matrices{ind2}-A(:,:,ind2)*B) == 0;

    % Two gridvar objects where one has only one gridpoint (swapped
    % arguments)
    C3 = Bg*Ag;

    ind3 = randi(Ag.Ng);
    check(3) = norm(C3.matrices{ind3}-B*A(:,:,ind3)) == 0;

    % Gridvar and contant
    C4 = Ag*B;

    ind4 = randi(Ag.Ng);
    check(4) = norm(C4.matrices{ind4}-A(:,:,ind4)*B) == 0;

    % Constant and gridvar
    C5 = B*Ag;

    ind5 = randi(Ag.Ng);
    check(5) = norm(C5.matrices{ind5}-B*A(:,:,ind5)) == 0;

    % Check
    verifyTrue(testCase, all(check)); 
end

function testTimes(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    rng(1);

    % Two gridvar objects with same number of gridpoints
    C1 = Ag.*Ag;

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-A(:,:,ind1).*A(:,:,ind1)) == 0;

    % Two gridvar objects where one has only one gridpoint
    C2 = Ag.*Bg';

    ind2 = randi(Ag.Ng);
    check(2) = norm(C2.matrices{ind2}-A(:,:,ind2).*B') == 0;

    % Two gridvar objects where one has only one gridpoint (swapped
    % arguments)
    C3 = Bg.*Ag';

    ind3 = randi(Ag.Ng);
    check(3) = norm(C3.matrices{ind3}-B.*A(:,:,ind3)') == 0;

    % Gridvar and contant
    C4 = Ag.*B';

    ind4 = randi(Ag.Ng);
    check(4) = norm(C4.matrices{ind4}-A(:,:,ind4).*B') == 0;

    % Constant and gridvar
    C5 = B.*Ag';

    ind5 = randi(Ag.Ng);
    check(5) = norm(C5.matrices{ind5}-B.*A(:,:,ind5)') == 0;

    % Check
    verifyTrue(testCase, all(check)); 
end

function testVariousFun(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    % Make square and same size
    n = min([size(A,[1 2]),size(B,[1 2])]);
    A = A(1:n,1:n,:);
    Ag = Ag(1:n,1:n);

    B = B(1:n,1:n,:);
    Bg = Bg(1:n,1:n);

    rng(1);

    funcs = {@plus, @ldivide, @rdivide, @mldivide, @mrdivide};

    for i = 1:numel(funcs)
        fun = funcs{i};

        % Two gridvar objects with same number of gridpoints
        C1 = fun(Ag, Ag);
    
        ind1 = randi(Ag.Ng);
        check(1) = norm(C1.matrices{ind1} - fun(A(:,:,ind1), A(:,:,ind1))) == 0;
    
        % Two gridvar objects where one has only one gridpoint
        C2 = fun(Ag, Bg);
    
        ind2 = randi(Ag.Ng);
        check(2) = norm(C2.matrices{ind2} - fun(A(:,:,ind2), B)) == 0;
    
        % Two gridvar objects where one has only one gridpoint (swapped
        % arguments)
        C3 = fun(Bg, Ag);
    
        ind3 = randi(Ag.Ng);
        check(3) = norm(C3.matrices{ind3} - fun(B, A(:,:,ind3))) == 0;
    
        % Gridvar and contant
        C4 = fun(Ag, B);
    
        ind4 = randi(Ag.Ng);
        check(4) = norm(C4.matrices{ind4} - fun(A(:,:,ind4), B)) == 0;
    
        % Constant and gridvar
        C5 = fun(B, Ag);
    
        ind5 = randi(Ag.Ng);
        check(5) = norm(C5.matrices{ind5} - fun(B, A(:,:,ind5))) == 0;
    
        % Check
        verifyTrue(testCase, all(check)); 
    end
end

function testPlus(testCase)
    A = testCase.TestData.A;
    B = testCase.TestData.B;
    Ag = testCase.TestData.Ag;
    Bg = testCase.TestData.Bg;

    % Make square and same size
    n = min([size(A,[1 2]),size(B,[1 2])]);
    A = A(1:n,1:n,:);
    Ag = Ag(1:n,1:n);

    B = B(1:n,1:n,:);
    Bg = Bg(1:n,1:n);

    rng(1);

    % Two gridvar objects with same number of gridpoints
    C1 = Ag+Ag;

    ind1 = randi(Ag.Ng);
    check(1) = norm(C1.matrices{ind1}-(A(:,:,ind1)+A(:,:,ind1))) == 0;

    % Two gridvar objects where one has only one gridpoint
    C2 = Ag+Bg;

    ind2 = randi(Ag.Ng);
    check(2) = norm(C2.matrices{ind2}-(A(:,:,ind2)+B)) == 0;

    % Two gridvar objects where one has only one gridpoint (swapped
    % arguments)
    C3 = Bg+Ag;

    ind3 = randi(Ag.Ng);
    check(3) = norm(C3.matrices{ind3}-(B+A(:,:,ind3))) == 0;

    % Gridvar and contant
    C4 = Ag+B;

    ind4 = randi(Ag.Ng);
    check(4) = norm(C4.matrices{ind4}-(A(:,:,ind4)+B)) == 0;

    % Constant and gridvar
    C5 = B+Ag;

    ind5 = randi(Ag.Ng);
    check(5) = norm(C5.matrices{ind5}-(B+A(:,:,ind5))) == 0;

    % Check
    verifyTrue(testCase, all(check)); 
end

function testLe(testCase)
    % This test also verifies value/double
    C2 = testCase.TestData.C2;
    D = testCase.TestData.D;
    Cg = testCase.TestData.Cg;
    Dg = testCase.TestData.Dg;

    % Two gridvar objects with same number of gridpoints
    LMI = Dg'*Cg+Cg*Dg <= -eye(3)*1e-8;

    optimize(LMI,[],sdpsettings('solver','sdpt3','verbose',0));
    E1 = getMatrices(value(Cg));

    % Manually
    LMI = [];
    for i = 1:size(C2,3)
        LMI = LMI + (D(:,:,i)'*C2(:,:,i)+C2(:,:,i)*D(:,:,i) <= -eye(3)*1e-8);
    end

    optimize(LMI,[],sdpsettings('solver','sdpt3','verbose',0));
    E2 = value(C2);

    % Check
    verifyTrue(testCase, all(E1 - E2 == 0, 'all')); 
end

function testLt(testCase)
    C2 = testCase.TestData.C2;
    D = testCase.TestData.D;
    Cg = testCase.TestData.Cg;
    Dg = testCase.TestData.Dg;

    % Two gridvar objects with same number of gridpoints
    LMI = Dg'*Cg+Cg*Dg < 0;

    optimize(LMI,[],sdpsettings('solver','sdpt3','verbose',0));
    E1 = getMatrices(value(Cg));

    % Manually
    LMI = [];
    for i = 1:size(C2,3)
        % Use strict inequality, i.e. <=, as that is what gridvar will use
        % and otherwise YALMIP starts spamming cat pictures
        LMI = LMI + (D(:,:,i)'*C2(:,:,i)+C2(:,:,i)*D(:,:,i) <= 0);
    end

    optimize(LMI,[],sdpsettings('solver','sdpt3','verbose',0));
    E2 = value(C2);

    % Check
    verifyTrue(testCase, all(E1 - E2 == 0, 'all')); 
end