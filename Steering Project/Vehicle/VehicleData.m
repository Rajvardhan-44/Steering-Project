function car = VehicleData()

car.m = 375;          % kg
car.g = 9.81;         % m/s^2

car.wd = 0.40;        % Front weight distribution

h = 150;              % CG height in mm
l = 1574.8;           % wheelbase in mm
t = 1219.2;           % track width in mm

w = 205;              % tyre width in mm
d = 406.4;            % tyre diameter in mm

r = 550;              % rack length in mm

% car.tr = 5;           % tightest radius car should take (in m)
% car.toe = 0.1;        % toe of each front wheel in straight (magnitude)

car.kf = 349;         % front roll stiffness in Nm/deg
car.kr = 285;         % rear roll stiffness in Nm/deg

% Aerodynamics
car.dens = 1.23;      % air density (SI Unit)

% Front Wing Specs
car.fcdg = 0.2;       % front wing co-efficient of drag
car.fcdf = 0.1;       % front wing co-efficient of downforce
car.farea = 0.05;     % front wing area (in meter square)

car.fwh = 0.1;        % distance between drag force and CG (m)
car.fwa = 0.2;        % distance between downforce and CG (m)

% Rear Wing Specs
car.rcdg = 0.2;       % rear wing co-efficient of drag
car.rcdf = 0.1;       % rear wing co-efficient of downforce
car.rarea = 0.05;     % rear wing area (in meter square)

car.rwh = 0.1;        % distance between drag force and CG (m)
car.rwb = 0.2;        % distance between downforce and CG (m)


%% SI Unit Conversion
car.h = h/1000;
car.l = l/1000;
car.t = t/1000;

car.w = w/1000;
car.d = d/1000;

car.r = r/1000;

car.a = (1 - car.wd) * car.l;           % distance between front axle and CG
car.b = car.wd * car.l;                 % distance between rear axle and CG


% Corner Radius (m) to simulate
car.radius = [...
    5                               % Tightest corner car should take
    6
    7
    8
    9
    10
    12
    14
    16
    18
    20
    25
    30
    40
    50
    70
    100
    150
    200
    ];


end