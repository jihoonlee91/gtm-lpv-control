close all

height_ft = 0:1000:30000;
[T, a, P, rho] = atmosisa(convlength(height_ft,'ft','m'));
[T0, a0, P0, rho0] = atmosisa(0);

figure, plot(height_ft,rho/rho0,height_ft,P/P0,'--')
xlabel('Height [ft]'), ylabel('Ratio')
legend('Density Ratio \rho/\rho_0','Pressure Ratio P/P_0')
grid on, grid minor

exportgraphics(gcf,'DensityPressure.pdf')