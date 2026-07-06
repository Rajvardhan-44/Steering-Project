function Angle = FindAngle(Result,car)

%-------------------------------------------------------------
% Verify whether rear slip angles correspond to assumed radius
%
% INPUTS
% Result : output of FindSpeed.m
% car    : VehicleData structure
%
% OUTPUT
% Angle.ICx
% Angle.ICy
% Angle.Rcalc
%
% Origin: Rear Differential
% X-axis: right
% Y-axis: front
%
% Angles are measured anti-clockwise from +X-axis
%
%-------------------------------------------------------------

% Number of cases
n = length(Result.Radius);

% Rear wheel coordinates (Rear Differential is origin)
% xRO =  car.t/2;    yRO = 0;
% xRI = -car.t/2;    yRI = 0;

% Setup
Angle.ICx   = zeros(n,1);
Angle.ICy   = zeros(n,1);

Angle.Rcalc = zeros(n,1);
Angle.Error = zeros(n,1);

%% Loop
for i = 1:n

    % Rear slip angles (stored as +ve in Radians)
    alphaRI = deg2rad(Result.alphaRI(i));
    alphaRO = deg2rad(Result.alphaRO(i));

    % Angle between -X and Actual velocity direction (+ve)
    thetaRI = alphaRI;
    thetaRO = alphaRO;
    
    % IC Co-ordinates (All are +ve)
    % But IC is in 2nd Quadrant
    ICx = ( car.t/2*(tan(thetaRI) + tan(thetaRO))) / (tan(thetaRI) - tan(thetaRO));
    ICy = tan(thetaRI) * (ICx - car.t/2);

    % %% Solve intersection
    % 
    % A = [ uRI(1)  -uRO(1)
    %       uRI(2)  -uRO(2)];
    % 
    % b = (pRI-pRO)';
    % 
    % s = A\b;
    % 
    % IC = pRO + s(1)*uRI;
    % 
    % Angle.ICx(i) = IC(1);
    % Angle.ICy(i) = IC(2);

    %% Radius from IC to CG
    rcalc = sqrt( ICx^(2)+(ICy-car.b)^(2)) ;
    Angle.Rcalc(i) = rcalc;
    % Angle.Rcalc(i) = hypot(IC(1),IC(2));

    %% Error

    Angle.Error(i) = Angle.Rcalc(i)-Result.Radius(i);

end

%% Plot

figure
plot(Result.Radius,Result.Radius,'r--','LineWidth',2)
hold on
plot(Result.Radius,Angle.Rcalc,'w','LineWidth',2)

grid on
xlabel('Assumed Radius (m)')
ylabel('Calculated Radius (m)')
legend('Ideal','From Rear Slip Angles','Location','best')

title('Radius Verification')

%% Error Plot

figure

plot(Result.Radius,Angle.Error,'r','LineWidth',2)

grid on

xlabel('Assumed Radius (m)')
ylabel('Radius Error (m)')

title('Radius Error')

end