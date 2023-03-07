%%
etal = linspace(-25,0,100);
dinertia = zeros(100,6);
dCG = zeros(100,3);
for i = 1:100
    eta = [etal(i);0];
    [dinertia(i,:),dCG(i,:)]  = morphing_effect(eta);
end

figure('units','centimeters','position',[0 12 22 7]);

subplot(1,3,1)
plot(etal,dinertia(:,1),'b',etal,dinertia(:,2),'--k',etal,dinertia(:,3),'-.r')
xlabel('\eta_l [%]'), ylabel('Momemt of inertia [slug·ft^2]'), grid on, grid minor, ylim padded, xlim tight
legend('\DeltaJ_{x}','\DeltaJ_{y}','\DeltaJ_{z}','location','southeast')

subplot(1,3,2)
plot(etal,dinertia(:,6),'b',etal,dinertia(:,5),'--k',etal,dinertia(:,4),'-.r')
xlabel('\eta_l [%]'), ylabel('Product of inertia [slug·ft^2]'), grid on, grid minor, ylim padded, xlim tight
legend('\DeltaJ_{xy}','\DeltaJ_{yz}','\DeltaJ_{zx}','location','southeast')

subplot(1,3,3)
plot(etal,dCG(:,1),'b',etal,dCG(:,2),'--k',etal,dCG(:,3),'-.r')
xlabel('\eta_l [%]'), ylabel('Center of mass [ft]'), grid on, grid minor, ylim padded, xlim tight
legend('\Deltax_{cm}','\Deltay_{cm}','\Deltaz_{cm}','location','northeast')

exportgraphics(gcf,'figure/etal_property_effect.png','resolution',600)

%%
etas = linspace(-25,0,100);
dinertia = zeros(100,6);
dCG = zeros(100,3);
for i = 1:100
    eta = etas(i)*[1;1];
    [dinertia(i,:),dCG(i,:)]  = morphing_effect(eta);
end

figure('units','centimeters','position',[0 2 22 7]);

subplot(1,3,1)
plot(etal,dinertia(:,1),'b',etal,dinertia(:,2),'--k',etal,dinertia(:,3),'-.r')
xlabel('\eta_s [%]'), ylabel('Momemt of inertia [slug·ft^2]'), grid on, grid minor, ylim padded, xlim tight
legend('\DeltaJ_{x}','\DeltaJ_{y}','\DeltaJ_{z}','location','southeast')

subplot(1,3,2)
plot(etal,dinertia(:,6),'b',etal,dinertia(:,5),'--k',etal,dinertia(:,4),'-.r')
xlabel('\eta_s [%]'), ylabel('Product of inertia [slug·ft^2]'), grid on, grid minor, ylim padded, xlim tight
legend('\DeltaJ_{xy}','\DeltaJ_{yz}','\DeltaJ_{zx}','location','southeast')

subplot(1,3,3)
plot(etal,dCG(:,1),'b',etal,dCG(:,2),'--k',etal,dCG(:,3),'-.r')
xlabel('\eta_s [%]'), ylabel('Center of mass [ft]'), grid on, grid minor, ylim padded, xlim tight
legend('\Deltax_{cm}','\Deltay_{cm}','\Deltaz_{cm}','location','northeast')

exportgraphics(gcf,'figure/etas_property_effect.png','resolution',600)

%%
etal = linspace(-25,0,100);
dinertia = zeros(100,6);
dCG = zeros(100,3);
for i = 1:100
    eta = [etal(i);0];
    [dinertia(i,:),dCG(i,:)]  = morphing_effect(eta);
end
dinertia = dinertia./[1.221 4.655 5.587 0.274 0.000 0.006]*100;
dCG = dCG/0.9153*100;

figure('units','centimeters','position',[0 12 22 7]);

subplot(1,2,1)
plot(etal,dinertia(:,1),'b',etal,dinertia(:,2),'--k',etal,dinertia(:,3),'-.r')
xlabel('\eta_l [%]'), ylabel('Momemt of inertia variation [%]'), grid on, grid minor, ylim padded, xlim tight
legend('$\Delta J_{x}/\bar{J}_x$','$\Delta J_{y}/\bar{J}_y$','$\Delta J_{z}/\bar{J}_z$','interpreter','latex','location','southeast')

subplot(1,2,2)
plot(etal,dCG(:,1),'b',etal,dCG(:,2),'--k',etal,dCG(:,3),'-.r')
xlabel('\eta_l [%]'), ylabel('Center of mass variation [%]'), grid on, grid minor, ylim padded, xlim tight
legend('$\Delta x_{cm}/\bar{c}$','$\Delta y_{cm}/\bar{c}$','$\Delta z_{cm}/\bar{c}$','interpreter','latex','location','northeast')

exportgraphics(gcf,'figure/etal_property_effect_ratio.png','resolution',600)

%%
etas = linspace(-25,0,100);
dinertia = zeros(100,6);
dCG = zeros(100,3);
for i = 1:100
    eta = etas(i)*[1;1];
    [dinertia(i,:),dCG(i,:)]  = morphing_effect(eta);
end

dinertia = dinertia./[1.221 4.655 5.587 0.274 0.000 0.006]*100;
dCG = dCG/0.9153*100;

figure('units','centimeters','position',[0 2 22 7]);

subplot(1,2,1)
plot(etal,dinertia(:,1),'b',etal,dinertia(:,2),'--k',etal,dinertia(:,3),'-.r')
xlabel('\eta_s [%]'), ylabel('Momemt of inertia variation [%]'), grid on, grid minor, ylim padded, xlim tight
legend('$\Delta J_{x}/\bar{J}_x$','$\Delta J_{y}/\bar{J}_y$','$\Delta J_{z}/\bar{J}_z$','interpreter','latex','location','southeast')

subplot(1,2,2)
plot(etal,dCG(:,1),'b',etal,dCG(:,2),'--k',etal,dCG(:,3),'-.r')
xlabel('\eta_s [%]'), ylabel('Center of mass variation [%]'), grid on, grid minor, ylim padded, xlim tight
legend('$\Delta x_{cm}/\bar{c}$','$\Delta y_{cm}/\bar{c}$','$\Delta z_{cm}/\bar{c}$','interpreter','latex','location','northeast')

exportgraphics(gcf,'figure/etas_property_effect_ratio.png','resolution',600)
