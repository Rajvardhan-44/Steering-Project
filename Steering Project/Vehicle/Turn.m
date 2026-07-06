function turn = Turn(car, move, accel, Front, Rear)

% Lateral Load Transfer
wt = accel * car.m * car.h / car.t;                 % total lateral load transfer
wtf = (car.kf / (car.kf + car.kr)) * wt;            % front lateral load transfer
wtr = (car.kr / (car.kf + car.kr)) * wt;            % rear lateral load transfer

% Vertical Loads
turn.Nfo = move.Wfe + wtf;                        % Front Outer 
turn.Nfi = move.Wfe - wtf;                        % Front Inner

turn.Nro = move.Wre + wtr;                        % Rear Outer
turn.Nri = move.Wre - wtr;                        % Rear Inner


%% Maximum Lateral Force Available

turn.FyMaxFO = abs(Pacejka96(-Front.alpha_limit, turn.Nfo, 0, Front));
turn.FyMaxFI = abs(Pacejka96(-Front.alpha_limit, turn.Nfi, 0, Front));

turn.FyMaxRO = abs(Pacejka96(-Rear.alpha_limit, turn.Nro, 0, Rear));
turn.FyMaxRI = abs(Pacejka96(-Rear.alpha_limit, turn.Nri, 0, Rear));

%% Total Available Force

turn.FyMaxFront = turn.FyMaxFO + turn.FyMaxFI;
turn.FyMaxRear  = turn.FyMaxRO + turn.FyMaxRI;

%% Required Axle Forces

turn.Fyf = car.wd * car.m * accel;

turn.Fyr = (1-car.wd) * car.m * accel;

%% Force Distribution

turn.Fyfo = turn.Fyf * turn.FyMaxFO/turn.FyMaxFront;                 % front outer needs to achieve this lateral force
turn.Fyfi = turn.Fyf * turn.FyMaxFI/turn.FyMaxFront;                 % front inner needs to achieve this lateral force

turn.Fyro = turn.Fyr * turn.FyMaxRO/turn.FyMaxRear;                  % rear outer needs to achieve this lateral force
turn.Fyri = turn.Fyr * turn.FyMaxRI/turn.FyMaxRear;                  % rear inner needs to achieve this lateral force


end