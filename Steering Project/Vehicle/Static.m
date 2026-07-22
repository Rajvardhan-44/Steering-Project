function static = Static(car)

Wf = car.wd*car.m*car.g;             % Front axle static load
static.Wfe = Wf/2;                   % Static Load on each front wheel

Wr = car.m*car.g - Wf;               % Rear axle static load
static.Wre = Wr/2;                   % Static Load on each rear wheel

end