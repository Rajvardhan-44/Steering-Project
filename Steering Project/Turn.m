function turn = Turn(car, move, a_long, a_lat)

%% Lateral Load Transfer
wt_lat = car.m*a_lat*car.h/car.t;

wt_lat_f = car.kf/(car.kf+car.kr)*wt_lat;
wt_lat_r = car.kr/(car.kf+car.kr)*wt_lat;


%% Longitudinal Load Transfer
wt_long = car.m*a_long*car.h/car.l;

wt_long_f = wt_long/2;
wt_long_r = wt_long/2;


%% Vertical Loads
turn.Nfo = move.Wfe + wt_lat_f + wt_long_f;
turn.Nfi = move.Wfe - wt_lat_f + wt_long_f;

turn.Nro = move.Wre + wt_lat_r - wt_long_r;
turn.Nri = move.Wre - wt_lat_r - wt_long_r;

end