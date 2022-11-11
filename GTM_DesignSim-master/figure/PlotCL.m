 CL = C6_bas.data(:,14,1).*sind(C6_bas.alpha)-C6_bas.data(:,14,3).*cosd(C6_bas.alpha);
 figure
 plot(C6_bas.alpha,CL)
 xlim([C6_bas.alpha(1) C6_bas.alpha(end)])
 grid