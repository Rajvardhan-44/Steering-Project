function PlotTyreCurve(tyre,Fz,gamma)

figure
hold on
grid on

for i=1:length(Fz)

    [alpha,Fy]=TyreData(tyre,Fz(i),gamma);

    plot(rad2deg(alpha),Fy,'LineWidth',2)

end

xlabel('Slip Angle (deg)')
ylabel('Lateral Force (N)')

title('Pacejka 96')

legend(string(Fz)+" N",'Location','best')

hold off

end