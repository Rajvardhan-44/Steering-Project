function move = Moving(car, v)
% v is in m/s

%% Aero Effects

% Drag Effects
dragf = 0.5 * car.dens * car.farea * car.fcdg * v^(2);             % Front wing Drag
dragr = 0.5 * car.dens * car.rarea * car.rcdg *v^(2);              % Rear wing Drag
dragm = car.rwh * dragr - car.fwh * dragf;                         % Moment about CG due to Drag

% Downforce Effects
dff = 0.5 * car.dens * car.farea * car.fcdf * v^(2);               % Front Wing Downforce
dfr = 0.5 * car.dens * car.rarea * car.rcdf * v^(2);               % Rear Wing Downforce


%% Vertical Loads at speed V

% Equations used to find move.Wf and move.Wr
% Wf + Wr = dff + dfr + m*g
% Wf*a + dragm + dfr*car.rwb = Wr*car.b + dff*car.fwa


move.Wf = ( (dff + dfr + car.m*car.g)*car.b + dff*car.fwa - dragm - dfr*car.rwb ) / (car.a+car.b);      % Front axle Load
move.Wfe = move.Wf/2;                                                                                   % Moving Load on each front wheel


move.Wr = car.m*car.g + dff + dfr - move.Wf;                        % Rear axle moving load
move.Wre = move.Wr / 2;                                             % Moving Load on each rear wheel


end