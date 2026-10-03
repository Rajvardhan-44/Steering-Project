function PlotMechanismFit(car, fitInfo)
%-------------------------------------------------------------
% Visualizes how well the synthesized mechanism matches the target
% delta_fi / delta_fo curves, radius by radius.
%-------------------------------------------------------------

figure('Name','Mechanism Fit','NumberTitle','off')

subplot(2,1,1)
hold on
grid on
plot(car.radius, fitInfo.targetInner,   'b--', 'LineWidth', 1.5)
plot(car.radius, fitInfo.achievedInner, 'b-',  'LineWidth', 2)
plot(car.radius, fitInfo.targetOuter,   'r--', 'LineWidth', 1.5)
plot(car.radius, fitInfo.achievedOuter, 'r-',  'LineWidth', 2)
if any(~fitInfo.reachable)
    plot(car.radius(~fitInfo.reachable), fitInfo.achievedInner(~fitInfo.reachable), ...
        'ko', 'MarkerSize', 8, 'LineWidth', 1.5)
end
xlabel('Corner Radius (m)')
ylabel('Steer Angle (deg)')
legend('Target Inner','Mechanism Inner','Target Outer','Mechanism Outer', ...
    'Location','best')
title('Steering Mechanism Fit vs Target')
hold off

subplot(2,1,2)
hold on
grid on
plot(car.radius, fitInfo.achievedOuter - fitInfo.targetOuter, 'r-', 'LineWidth', 1.5)
plot(car.radius, fitInfo.achievedInner - fitInfo.targetInner, 'b-', 'LineWidth', 1.5)
yline(0, 'k:')
xlabel('Corner Radius (m)')
ylabel('Error (deg)')
legend('Outer error','Inner error','Location','best')
title('Fit Error (Mechanism - Target)')
hold off

end