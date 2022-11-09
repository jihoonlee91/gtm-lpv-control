function Astab = myLPVstab2(Ae,sigA)
%function which stabilies a LPV denominator Ae by setting the root with the
%highest norm on the unit disc. 





Ab =Ae;
%compute the value of A for each scheduling parameter
A =Ab*sigA;
%compute the roots
for  itt = 1:size(A,2)
    r(itt,:) = (roots(A(:,itt))); %#ok<AGROW>
end
%sort them in order to find the maximal one
r1 =sort(abs(r),2,'descend');
r1 = r1(:,1);
indexmax =find(r1 ==max(r1));
indexmax=indexmax(1);
oldroots = r(indexmax,:);
%set it on the unit disc
newroots = min(abs(oldroots),0.999) .*exp(1i*angle(oldroots));
%case the roots were real set it to real again (sometimes it happens with
%numerical errors..
for itt = 1:length(oldroots)
    if isreal(oldroots(itt))
        newroots(itt) = real(newroots(itt));
    end
end

%recreate the coresponding polynom
%nonetheless the corresponding polynome has already a multiplication with
%the scheduling parameter at time tk..
NewA = poly(newroots);
%the ratio must be kept between the parameters
%compute hte diffrent ratios in the old A
oldA = Ae;
todiv = oldA(:,1);
todiv = repmat(todiv,1,size(oldA,2));

Coeffs = oldA./todiv;

%divide the new A by the scheduling parameters and the ratio to find an
%appropriate A stabilized
if size(NewA') == size(Coeffs*sigA(:,indexmax))
Astab = Coeffs.*repmat(NewA'./(Coeffs*sigA(:,indexmax)),1,size(oldA,2));
else
    disp('reinitialized');
    Astab = 0*Ae;
    Astab(1) =1;
end

