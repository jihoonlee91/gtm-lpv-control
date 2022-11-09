function [ae,be,ce,de]=obfextend(a,b,c,d,nrep)
%
%OBFEXTENd
%function [ae,be,ce,de]=obfextend(a,b,c,d,nrep)
%
%extend the square inner function defined by G(z)={a,b,c,d} to G(z)^{nrep}
%where {a,b,c,d} must be balanced!
%
%Note :  no checks are performed !!!!!!!!!
%
%input:     a  system matrix       dimension   n x n
%           b  input matrix                    n x m
%           c  output matrix                   m x n
%           d  feedthrough matrix              m x m
%        nrep  repetition number               1 x 1
%output:    ae system matrix                   (nrep x n) x (nrep x n)
%           be input matrix                    (nrep x n) x m
%           ce output matrix                    m x (nrep x n)
%           de feedthrough matrix               m x m

%Peter Heuberger
%November 12, 1999
%Version 1.0   under Matlab 5.3 (R11)
%October 18, 2002
%Version 1.1   removed output variable err
%April 4, 2003
%Version 1.2   removed use of function KRON

ra=size(a,1);


ae=zeros(nrep*ra);
be=ae(:,1);
ce=ae(1,:);
ae(1:ra,1:ra)=a;
be(1:ra)=b;
ce(end-ra+1:end)=c;
de=d;

for i=2:nrep
    ae((i-1)*ra+1:i*ra,1:i*ra)=[b*ce(end-(i-1)*ra+1:end) a];
    be((i-1)*ra+1:i*ra)=be((i-2)*ra+1:(i-1)*ra)*d;
    ce((nrep-i)*ra+1:(nrep-i+1)*ra)=d*ce((nrep-i+1)*ra+1:(nrep-i+2)*ra);
    de=de*d;
end

