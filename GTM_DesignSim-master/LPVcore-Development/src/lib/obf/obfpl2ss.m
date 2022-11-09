function [A1,B1,C1,D1,err]=obfpl2ss(pol)
%
% OBFPL2SS
% function [A1,B1,C1,D1,err]=obfpl2ss(pol)
% create  balanced state space form of inner function from poles in pol
% err=1 if poles not stable ( |pol|<1 required )
% leaves the order unaltered unless conjugate pairs are not consecutive
% hence [.1 .2-.3i .5 .2+.3i] is changed into [.1 .2-.3i .2+.3i .5]
%
% If pol is no vector, it is converted to a vector pol(:).
% On error the function exits with matlab's ERROR function 
% if nr of output arguments <5
% else function exits silently, with err>0:
%     err=1:  unstable poles
%     err=2,3:  unmatched conjugate pairs 

%Peter Heuberger
%October 11 , 1999
%Version 1.0
%differs from ortpl2ab in the order of the poles
%Version 1.1
%October 18 , 2002
%improved error handling

A1=[];B1=[];C1=[];D1=[];
err=0;

if nargin==0, return;end
polv=pol(:);
if isempty(polv), return; end

%check on stability
if max(abs(polv))>=1
    err=1;
    if nargout<5
        error('unstable poles defined');
    else
        return;
    end
end


%1 make sure complex poles are together paired
npol=length(polv);
pol2=polv;
counter=1;
while counter<npol+1 && ~err
   if isreal(pol2(counter))
      counter=counter+1;
   elseif counter==npol
      err=2;
   elseif pol2(counter+1)==conj(pol2(counter))
      counter=counter+2;
   else
      ind=find(pol2(counter+1:npol)==conj(pol2(counter)));
      if isempty(ind)
         err=3;
      else
         dum=pol2(counter+1);
         pol2(counter+1)=pol2(min(ind)+counter);
         pol2(min(ind)+counter)=dum;
         counter=counter+2;   
      end
   end
end

if err
    if nargout<5
        error('unmatched conjugate pairs defined');
    else
        return;
    end
end

%initialize
pb=pol2(1);
if isreal(pb)
   A1=pb;  B1=sqrt(1-pb*pb);  C1=B1; D1=-pb;
   counter=2;   
else
   p=real(pb);
   q=abs(imag(pb));
   pb2=p^2+q^2;
   A1=[2*p/(pb2+1) sqrt(1-4*p^2/(pb2+1)^2);-pb2*sqrt(1-4*p^2/(pb2+1)^2) pb2*2*p/(pb2+1)];
   B1=[0; sqrt(1-pb2^2)];
   C1=[sqrt((1-pb2^2)*(1-4*p^2/(pb2+1)^2)) -2*p*sqrt(1-pb2^2)/(1+pb2)];
   D1=pb2;
   counter=3;
end
while counter<npol+1
   pb=pol2(counter);
   if isreal(pb)
      A2=pb;  B2=sqrt(1-pb*pb);  C2=B2; D2=-pb;
      counter=counter+1;   
   else
      p=real(pb);
      q=abs(imag(pb));
      pb2=p^2+q^2;
      A2=[2*p/(pb2+1) sqrt(1-4*p^2/(pb2+1)^2);-pb2*sqrt(1-4*p^2/(pb2+1)^2) pb2*2*p/(pb2+1)];
      B2=[0; sqrt(1-pb2^2)];
      C2=[sqrt((1-pb2^2)*(1-4*p^2/(pb2+1)^2)) -2*p*sqrt(1-pb2^2)/(1+pb2)];
      D2=pb2;
      counter=counter+2;
   end   
   A1=[A1 zeros(size(A1,1),size(A2,2));B2*C1 A2]; %#ok<AGROW> 
   B1=[B1;B2*D1]; %#ok<AGROW> 
   C1=[D2*C1 C2];
   D1=D2*D1;
end
