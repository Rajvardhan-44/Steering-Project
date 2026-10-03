% function Result = Solver(car, Front, Rear)
% 
% %-------------------------------------------------------------
% % Finds the optimum IC location for a given radius.
% %
% % OUTPUT
% % Result.ICx
% % Result.ICy
% % Result.theta
% % Result.Vmax
% % Result.delta_fi
% % Result.delta_fo
% %-------------------------------------------------------------
% 
% 
% nRadius = numel(car.radius);
% 
% Result(nRadius) = struct('Radius',[], ...
%                          'ICx',[], ...
%                          'ICy',[], ...
%                          'theta',[], ...
%                          'Vmax',[], ...
%                          'delta_fi',[], ...
%                          'delta_fo',[]);
% 
% 
% % Initial speed
% PreviousVmax = 1;
% 
% 
% % For loop of each radius
% for r = 1:nRadius
% 
%     Radius = car.radius(r);
% 
%     if Radius <= car.l/2
%         error('Radius is too small.');
%     end
% 
% 
%     % Limits of IC arc
%     thetaMin = -asin((1-car.wd)*car.l / Radius);              % Front Axle -ve
%     thetaMax =  asin(car.wd*car.l / Radius);                  % Rear Axle  +ve
%     N = 100;
%     thetaList = linspace(thetaMax, thetaMin, N);
% 
% 
%     % Vmax of Previous Radius
%     StartSpeed = PreviousVmax;
% 
% 
%     % Best result
%     BestV = -inf;
%     BestTheta = NaN;
%     BestICx = NaN;
%     BestICy = NaN;
% 
% 
%     % Loop over IC locations
%     for i = 1:N
% 
%         theta = thetaList(i);
% 
%         ICx = -Radius*cos(theta);
%         ICy =  car.wd*car.l - Radius*sin(theta);
% 
% 
%         % Rear slip angles
%         alpha_ri = atan2(ICy , abs(ICx)-car.t/2);
%         alpha_ro = atan2(ICy , abs(ICx)+car.t/2);
% 
% 
%         % Velocity directions
%         vel_fi = atan2(car.l-ICy , abs(ICx)-car.t/2);
%         vel_fo = atan2(car.l-ICy , abs(ICx)+car.t/2);
% 
% 
%         % Speed search
%         speed = StartSpeed;
%         FineSearch = false;
% 
%         while true
% 
%             v = speed/3.6;
% 
%             a_net = v^2/Radius;
%             a_lat = a_net*cos(theta);
%             a_long = a_net*sin(theta);
% 
%             Limit = SolveEq(car,...
%                             v,...
%                             a_net,...
%                             a_lat,...
%                             a_long,...
%                             theta,...
%                             vel_fi,...
%                             vel_fo,...
%                             alpha_ri,...
%                             alpha_ro,...
%                             Front,...
%                             Rear);
% 
%             if ~Limit
%                 if ~FineSearch
%                     speed = max(speed-4, 0);
%                     FineSearch = true;
%                 else
%                     speed = max(speed-1, 0);
%                     break
%                 end
%             else
%                 if FineSearch
%                     speed = speed+1;
%                 else
%                     speed = speed+5;
%                 end
%             end
% 
% 
%             if speed > 500
%                 error('Solver exceeded 500 km/h. Something is wrong.');
%             end
% 
%         end
% 
% 
%         % Best IC ?
%         if speed >= BestV
%             BestV = speed;
%             BestTheta = theta;
%             BestICx = ICx;
%             BestICy = ICy;
%         end
% 
% 
%     end
% 
% 
%     fprintf(['Radius = %6.2f m | ', ...
%              'Vmax = %6.1f km/h | ', ...
%              'Theta = %7.3f deg\n'], Radius, BestV, rad2deg(BestTheta));
% 
%     % Save for next radius
%     PreviousVmax = BestV;
% 
% 
%     % Final steering angles
%     alpha_ri = atan2(BestICy , abs(BestICx)-car.t/2);
%     alpha_ro = atan2(BestICy , abs(BestICx)+car.t/2);
% 
%     vel_fi = atan2(car.l-BestICy , abs(BestICx)-car.t/2);
%     vel_fo = atan2(car.l-BestICy , abs(BestICx)+car.t/2);
% 
% 
%     % Calling FindDelta
%     v = BestV/3.6;
%     a_net  = v^2/Radius;
%     a_lat  = a_net*cos(BestTheta);
%     a_long = a_net*sin(BestTheta);
% 
%     [delta_fi,delta_fo] = FindDelta(car,...
%                                     v,...
%                                     a_net,...
%                                     a_lat,...
%                                     a_long,...
%                                     BestTheta,...
%                                     vel_fi,...
%                                     vel_fo,...
%                                     alpha_ri,...
%                                     alpha_ro,...
%                                     Front,...
%                                     Rear);
% 
% 
%     % Output
%     Result(r).Radius   = Radius;
%     Result(r).ICx      = BestICx;
%     Result(r).ICy      = BestICy;
% 
%     Result(r).theta    = BestTheta;
% 
%     Result(r).Vmax     = BestV;
% 
%     Result(r).delta_fi = delta_fi;
%     Result(r).delta_fo = delta_fo;
% 
% end
% 
% end



































% function Result = Solver(car, Front, Rear)
% 
% %-------------------------------------------------------------
% % Finds the optimum IC location for every radius.
% %
% % OUTPUT (per radius)
% % Result.Radius, ICx, ICy, theta
% % Result.Vmax      (km/h)
% % Result.delta_fi, delta_fo   (rad)
% % Result.alpha_fi, alpha_fo, alpha_ri, alpha_ro   (rad)
% %-------------------------------------------------------------
% 
% nRadius = numel(car.radius);
% N       = 60;                       % IC positions tried per radius
% 
% Result(nRadius) = struct('Radius',[], ...
%                          'ICx',[], ...
%                          'ICy',[], ...
%                          'theta',[], ...
%                          'Vmax',[], ...
%                          'delta_fi',[], ...
%                          'delta_fo',[], ...
%                          'alpha_fi',[], ...
%                          'alpha_fo',[], ...
%                          'alpha_ri',[], ...
%                          'alpha_ro',[]);
% 
% 
% for r = 1:nRadius
% 
%     Radius = car.radius(r);
% 
%     if Radius <= car.l/2
%         error('Radius is too small.');
%     end
% 
% 
%     % Limits of IC arc
%     thetaMin = -asin(car.a / Radius);           % Front axle line
%     thetaMax =  asin(car.b / Radius);           % Rear axle line
%     thetaList = linspace(thetaMax, thetaMin, N);
% 
% 
%     % Best result
%     Best   = [];
%     BestV  = -inf;
%     x_prev = [];
% 
% 
%     for i = 1:N
% 
%         theta = thetaList(i);
% 
%         [sol, ok] = SolveEq(car, Radius, theta, Front, Rear, x_prev);
% 
%         if ~ok
%             continue
%         end
% 
%         x_prev = [sol.v; sol.alpha_fi; sol.alpha_fo];
% 
%         if sol.v > BestV
%             BestV = sol.v;
%             Best  = sol;
%             Best.theta = theta;
%         end
% 
%     end
% 
% 
%     % Store
%     Result(r).Radius = Radius;
% 
%     if isempty(Best)
%         warning('Solver:NoSolution', ...
%                 'No feasible operating point for R = %.2f m.', Radius);
%         Result(r).ICx = NaN;      Result(r).ICy = NaN;
%         Result(r).theta = NaN;    Result(r).Vmax = NaN;
%         Result(r).delta_fi = NaN; Result(r).delta_fo = NaN;
%         Result(r).alpha_fi = NaN; Result(r).alpha_fo = NaN;
%         Result(r).alpha_ri = NaN; Result(r).alpha_ro = NaN;
%         continue
%     end
% 
%     Result(r).ICx      = Best.info.ICx;
%     Result(r).ICy      = Best.info.ICy;
%     Result(r).theta    = Best.theta;
%     Result(r).Vmax     = Best.v*3.6;                 % km/h
%     Result(r).delta_fi = Best.info.delta_fi;
%     Result(r).delta_fo = Best.info.delta_fo;
%     Result(r).alpha_fi = Best.alpha_fi;
%     Result(r).alpha_fo = Best.alpha_fo;
%     Result(r).alpha_ri = Best.info.alpha_ri;
%     Result(r).alpha_ro = Best.info.alpha_ro;
% 
%     fprintf(['Radius = %6.2f m | ', ...
%              'Vmax = %6.1f km/h | ', ...
%              'Theta = %7.3f deg\n'], Radius, Result(r).Vmax, rad2deg(Best.theta));
% 
% end
% 
% end























function Result = Solver(car, Front, Rear)
%-------------------------------------------------------------
% Finds the optimum IC location for every radius.
%
% IC search range: previously restricted to lie between the front and
% rear axle lines (the pure zero-slip Ackermann assumption). That's
% only exactly true at zero slip - under real slip angles the IC can
% genuinely sit ahead of the front axle or behind the rear axle. The
% only physical requirement for a left-hand corner is that the IC
% stays on the LEFT side of the car (ICx < 0), so the search now
% sweeps theta across the full left half-plane instead. Any IC
% position that isn't actually reachable (e.g. it would need more
% rear slip angle than the tyre can give) is discarded automatically
% by the Rear.alpha_limit check inside SolveEq - widening the search
% doesn't risk accepting unphysical results, it just stops
% pre-emptively ruling out valid ones.
%
% OUTPUT (per radius)
% Result.Radius, ICx, ICy, theta
% Result.Vmax      (km/h)
% Result.delta_fi, delta_fo   (rad)
% Result.alpha_fi, alpha_fo, alpha_ri, alpha_ro   (rad)
%-------------------------------------------------------------
nRadius = numel(car.radius);
N       = 121;                      % IC positions tried per radius
Result(nRadius) = struct('Radius',[], ...
    'ICx',[], ...
    'ICy',[], ...
    'theta',[], ...
    'Vmax',[], ...
    'delta_fi',[], ...
    'delta_fo',[], ...
    'alpha_fi',[], ...
    'alpha_fo',[], ...
    'alpha_ri',[], ...
    'alpha_ro',[]);
for r = 1:nRadius
    Radius = car.radius(r);
    if Radius <= 0
        error('Radius must be positive.');
    end
    % Limits of IC arc: the whole left half-plane, short of the
    % vehicle centerline singularity at theta = +-90 deg.
    thetaLimit = deg2rad(89);   % EDIT: margin from the +-90 deg singularity
    thetaMin = -thetaLimit;
    thetaMax =  thetaLimit;

    % Non-uniform sampling: pack points near theta = 0 (where the
    % equilibrium IC sits for most radii, especially large ones, since
    % the old axle-line bounds used to shrink toward 0 there anyway)
    % and thin them out near the rarely-useful extremes.
    p = linspace(1, -1, N);
    thetaList = thetaLimit * sign(p) .* p.^2;
    % Best result
    Best   = [];
    BestV  = -inf;
    x_prev = [];
    for i = 1:N
        theta = thetaList(i);
        [sol, ok] = SolveEq(car, Radius, theta, Front, Rear, x_prev);
        if ~ok
            continue
        end
        x_prev = [sol.v; sol.alpha_fi; sol.alpha_fo];
        if sol.v > BestV
            BestV = sol.v;
            Best  = sol;
            Best.theta = theta;
        end
    end
    % Store
    Result(r).Radius = Radius;
    if isempty(Best)
        warning('Solver:NoSolution', ...
            'No feasible operating point for R = %.2f m.', Radius);
        Result(r).ICx = NaN;      Result(r).ICy = NaN;
        Result(r).theta = NaN;    Result(r).Vmax = NaN;
        Result(r).delta_fi = NaN; Result(r).delta_fo = NaN;
        Result(r).alpha_fi = NaN; Result(r).alpha_fo = NaN;
        Result(r).alpha_ri = NaN; Result(r).alpha_ro = NaN;
        continue
    end
    Result(r).ICx      = Best.info.ICx;
    Result(r).ICy      = Best.info.ICy;
    Result(r).theta    = Best.theta;
    Result(r).Vmax     = Best.v*3.6;                 % km/h
    Result(r).delta_fi = Best.info.delta_fi;
    Result(r).delta_fo = Best.info.delta_fo;
    Result(r).alpha_fi = Best.alpha_fi;
    Result(r).alpha_fo = Best.alpha_fo;
    Result(r).alpha_ri = Best.info.alpha_ri;
    Result(r).alpha_ro = Best.info.alpha_ro;
    fprintf(['Radius = %6.2f m | ', ...
        'Vmax = %6.1f km/h | ', ...
        'Theta = %7.3f deg\n'], Radius, Result(r).Vmax, rad2deg(Best.theta));
end
end