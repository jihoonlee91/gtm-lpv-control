set(0,'DefaultLineLineWidth',1)

alpha = dC6_damage.bas.alpha;
beta = dC6_damage.bas.beta;
dCX = dC6_damage.bas.data(:,7,4,1);
dCY = dC6_damage.bas.data(:,7,4,2);
dCZ = dC6_damage.bas.data(:,7,4,3);
dCl = dC6_damage.bas.data(:,7,4,4);
dCm = dC6_damage.bas.data(:,7,4,5);
dCn = dC6_damage.bas.data(:,7,4,6);

% left wingtip loss (wrt alpha and beta)
[Alpha,Beta] = meshgrid(alpha,beta);
figure('units','centimeters','position',[13 2 20 20]);
subplot(3,2,1), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,1)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCX'), grid on, grid minor, box on, view(0,0)
subplot(3,2,3), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,2)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCY'), grid on, grid minor, box on, view(0,0)
subplot(3,2,5), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,3)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCZ'), grid on, grid minor, box on, view(0,0)
subplot(3,2,2), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,4)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCl'), grid on, grid minor, box on, view(0,0)
subplot(3,2,4), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,5)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCm'), grid on, grid minor, box on, view(0,0)
subplot(3,2,6), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,6)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCn'), grid on, grid minor, box on, view(0,0)

figure('units','centimeters','position',[13 2 20 20]);
subplot(3,2,1), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,1)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCX'), grid on, grid minor, box on, view(90,0)
subplot(3,2,3), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,2)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCY'), grid on, grid minor, box on, view(90,0)
subplot(3,2,5), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,3)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCZ'), grid on, grid minor, box on, view(90,0)
subplot(3,2,2), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,4)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCl'), grid on, grid minor, box on, view(90,0)
subplot(3,2,4), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,5)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCm'), grid on, grid minor, box on, view(90,0)
subplot(3,2,6), s = surf(Alpha,Beta,squeeze(dC6_damage.bas.data(:,:,4,6)).'); s.EdgeColor = 'flat'; xlabel('\alpha [deg]'), ylabel('\beta [deg]'), zlabel('dCn'), grid on, grid minor, box on, view(90,0)

% left wingtip loss (beta = 0)
figure('units','centimeters','position',[13 2 13 20]);
subplot(3,2,1), plot(alpha,dCX), xlabel('Angle of Attack [deg]'), ylabel('dCX'), grid on, grid minor
subplot(3,2,2), plot(alpha,dCY), xlabel('Angle of Attack [deg]'), ylabel('dCY'), grid on, grid minor
subplot(3,2,5), plot(alpha,dCZ), xlabel('Angle of Attack [deg]'), ylabel('dCZ'), grid on, grid minor
subplot(3,2,2), plot(alpha,dCl), xlabel('Angle of Attack [deg]'), ylabel('dCl'), grid on, grid minor
subplot(3,2,4), plot(alpha,dCm), xlabel('Angle of Attack [deg]'), ylabel('dCm'), grid on, grid minor
subplot(3,2,6), plot(alpha,dCn), xlabel('Angle of Attack [deg]'), ylabel('dCn'), grid on, grid minor

% eta_s vs eta_a (wrt alpha, beta = 0)
figure('units','centimeters','position',[13 2 13 20]);
subplot(3,2,1), plot(alpha,dCX*2,alpha,dCX*0,'--'), xlabel('Angle of Attack [deg]'), ylabel('dCX'), legend('Both Retracted','Left Retracted','location','best'), grid on, grid minor
subplot(3,2,3), plot(alpha,dCY*0,alpha,dCY*1,'--'), xlabel('Angle of Attack [deg]'), ylabel('dCY'), legend('\eta_s=-25,\eta_a=0','\eta_s=-12.5,\eta_a=12.5','location','best'), grid on, grid minor
subplot(3,2,5), plot(alpha,dCZ*2,alpha,dCZ*0,'--'), xlabel('Angle of Attack [deg]'), ylabel('dCZ'), legend('\eta_s=-25,\eta_a=0','\eta_s=-12.5,\eta_a=12.5','location','best'), grid on, grid minor
subplot(3,2,2), plot(alpha,dCl*0,alpha,dCl*1,'--'), xlabel('Angle of Attack [deg]'), ylabel('dCl'), legend('\eta_s=-25,\eta_a=0','\eta_s=-12.5,\eta_a=12.5','location','best'), grid on, grid minor
subplot(3,2,4), plot(alpha,dCm*2,alpha,dCm*0,'--'), xlabel('Angle of Attack [deg]'), ylabel('dCm'), legend('\eta_s=-25,\eta_a=0','\eta_s=-12.5,\eta_a=12.5','location','best'), grid on, grid minor
subplot(3,2,6), plot(alpha,dCn*0,alpha,dCn*1,'--'), xlabel('Angle of Attack [deg]'), ylabel('dCn'), legend('\eta_s=-25,\eta_a=0','\eta_s=-12.5,\eta_a=12.5','location','best'), grid on, grid minor

% eta_s vs eta_a (alpha = 10, wrt beta)