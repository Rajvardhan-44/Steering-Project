function static = Static()

static.Wf = wd*m*g;    % Front axle static load
static.Wfe = Wf/2;     % Static Load on each front wheel

static.Wr = m*g - Wf;  % Rear axle static load
static.Wre = Wr/2;     % Static Load on each rear wheel

end