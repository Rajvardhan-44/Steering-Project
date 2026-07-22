function Info(car, Result)

%-------------------------------------------------------------
% Displays Results
%
% Figure 1 : IC locations with vehicle sketch
% Figure 2 : Maximum speed vs Radius
% Figure 3 : Steering angles vs Radius
%-------------------------------------------------------------


% Extract Data
Radius  = [Result.Radius]';
ICx     = [Result.ICx]';
ICy     = [Result.ICy]';
Vmax    = [Result.Vmax]';
delta_fi = abs(rad2deg([Result.delta_fi]'));
delta_fo = abs(rad2deg([Result.delta_fo]'));


%% Figure 1 : IC Locations
figure('Name', 'IC vs R',...
       'NumberTitle', 'off',...
       'Color', 'w');

hold on
axis equal
grid on
box on

title('Best IC for each Radius')

xlabel('X (m)')
ylabel('Y (m)')

% ---------------- Setup ----------------
rearX  = 0;
rearY  = 0;

frontX = 0;
frontY = car.l;

halft = car.t/2;

tyreW = car.w;
tyreD = car.d;


% ---------------- Tyre ----------------
% Rear Left
rectangle('Position',...
                     [rearX - halft - tyreW/2,...
                      rearY - tyreD/2,...
                      tyreW,...
                      tyreD],...
                             'EdgeColor', [0.3 0.3 0.3],...
                             'LineWidth', 1.5);

% Rear Right
rectangle('Position',...
                     [rearX + halft - tyreW/2,...
                      rearY - tyreD/2,...
                      tyreW,...
                      tyreD],...
                             'EdgeColor', [0.3 0.3 0.3],...
                             'LineWidth', 1.5);

% Front Left
rectangle('Position',...
                     [frontX - halft - tyreW/2,...
                      frontY - tyreD/2,...
                      tyreW,...
                      tyreD],...
                             'EdgeColor', [0.3 0.3 0.3],...
                             'LineWidth', 1.5);

% Front Right
rectangle('Position',...
                     [frontX + halft - tyreW/2,...
                      frontY - tyreD/2,...
                      tyreW,...
                      tyreD],...
                             'EdgeColor', [0.3 0.3 0.3],...
                             'LineWidth', 1.5);



% ---------------- Chassis ----------------
% Rear axle
plot([rearX-halft rearX+halft],...
     [rearY       rearY],...
                         '--', 'Color', [0.75 0.75 0.75]);

% Drive axle
plot([rearX frontX], ...
     [rearY frontY],...
                    '--', 'Color', [0.75 0.75 0.75]);

% Front axle
plot([frontX-halft frontX+halft], ...
     [frontY       frontY],...
                    '--', 'Color', [0.75 0.75 0.75]);


% ---------------- CG ----------------
CGx = 0;
CGy = car.b;

plot(CGx, CGy,...
              'ko', 'MarkerFaceColor', 'k',...
              'MarkerSize', 6);

text(CGx+0.05, CGy, 'CG');


% ---------------- IC Arcs ----------------
for i = 1:length(Result)

    R = Radius(i);

    % Arc from rear axle limit to front axle limit
    thetaArc = linspace(asin(car.b / R), -asin(car.a / R), 150);

    xArc = CGx - R*cos(thetaArc);
    yArc = CGy - R*sin(thetaArc);

    plot(xArc, yArc, '--', 'Color', [0.80 0.80 0.80]);
    text(ICx(i)-0.12, ICy(i),...
                             sprintf('%gm', Radius(i)),...
                            'FontSize', 8,...
                            'HorizontalAlignment', 'right');

    

end


% IC Points
plot(ICx, ICy,...
              'ko', 'MarkerFaceColor', 'k',...
              'MarkerSize', 5);


% ---------------- View ----------------
% xlim('auto')
% ylim('auto')


%% Figure 2 : Vmax vs Radius
figure('Name', 'Vmax vs R',...
       'NumberTitle', 'off',...
       'Color', 'w');

plot(Radius, Vmax,...
     '-k', 'LineWidth', 2,...
     'MarkerFaceColor', 'k');

grid on
box on

xlabel('Corner Radius (m)')
ylabel('Maximum Speed (km/h)')
title('Maximum Speed vs Corner Radius')


%% Figure 3 : Steering Angles
figure('Name','delta vs R',...
       'NumberTitle', 'off',...
       'Color', 'w');

hold on

plot(Radius, delta_fi,...
     '-b', 'LineWidth', 2,...
     'MarkerFaceColor', 'r');

plot(Radius, delta_fo,...
     '-r', 'LineWidth', 2,...
     'MarkerFaceColor', 'b');

grid on
box on

xlabel('Corner Radius (m)')
ylabel('Steering Angle Magnitude (deg)')

legend('| \delta_{fi} |',...
       '| \delta_{fo} |',...
       'Location','best');

title('Steering Angle vs Corner Radius')

end