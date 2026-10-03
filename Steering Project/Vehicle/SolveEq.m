% function Limit = SolveEq(car,...
%                          v,...
%                          a_net,...
%                          a_lat,...
%                          a_long,...
%                          theta,...
%                          vel_fi,...
%                          vel_fo,...
%                          alpha_ri,...
%                          alpha_ro,...
%                          Front,...
%                          Rear)
% 
% 
% %-------------------------------------------------------------
% % Checks whether a given speed is physically possible.
% %
% % OUTPUT
% %   Limit = true   -> Car can sustain this speed within tire limits
% %   Limit = false  -> Impossible (or exceeds 10°/tire limits)
% %-------------------------------------------------------------
% 
% 
% 
% % Checking Rear Slip angles are in limit or not
% if abs(alpha_ri) > Rear.alpha_limit || abs(alpha_ro) > Rear.alpha_limit
%     Limit = false;
%     return
% end
% 
% 
% % Moment function
% M = @(rx,ry,Fx,Fy)  rx.*Fy - ry.*Fx;
% 
% 
% % Vehicle Loads
% move = Moving(car, v);
% turn = Turn(car, move, a_long, a_lat);
% 
% 
% % Wheel Locations
% r_fix = -car.t/2;
% r_fiy =  car.a;
% 
% r_fox = car.t/2;
% r_foy = car.a;
% 
% r_rix = -car.t/2;
% r_riy = -car.b;
% 
% r_rox =  car.t/2;
% r_roy = -car.b;
% 
% 
% % Rear Tyre Forces
% Fp_ri = abs(Pacejka96(alpha_ri, turn.Nri, 0, Rear));
% Fp_ro = abs(Pacejka96(alpha_ro, turn.Nro, 0, Rear));
% 
% 
% % Unknown front slip angles solver
% fun = @(x) Residual(x);
% 
% % Starts the guess in the middle of the safe zone
% x0 = [Front.alpha_limit/2;
%       Front.alpha_limit/2];
% 
% % Prevents solver from ever exceeding the front tire limit
% lb = [0; 
%       0];
% 
% ub = [Front.alpha_limit; 
%       Front.alpha_limit];
% 
% options = optimoptions('lsqnonlin',...
%                        'Display','off',...
%                        'FunctionTolerance',1e-8,...
%                        'StepTolerance',1e-8);
% 
% 
% [~, ~, residual, exitflag] = lsqnonlin(fun, x0, lb, ub, options);
% 
% 
% % Check Solver Success & Convergence Sanity
% % exitflag <= 0 means solver failed.
% % norm(residual) > 1e-3 ensures the mathematical forces actually balance.
% if exitflag <= 0 || norm(residual) > 1e-3
%     fprintf('SolveEq failed: v = %.2f km/h, exitflag = %d, residual = %.6f\n',...
%         v*3.6, exitflag, norm(residual));
%     Limit = false;
%     return
% end
% 
% Limit = true;
% 
% 
%     function F = Residual(x)
%         alpha_fi = x(1);
%         alpha_fo = x(2);
% 
%         % Steering Angles
%         delta_fi = vel_fi + alpha_fi;
%         delta_fo = vel_fo + alpha_fo;
% 
%         % Front Pacejka Forces
%         Fp_fi = abs(Pacejka96(alpha_fi, turn.Nfi, 0, Front));
%         Fp_fo = abs(Pacejka96(alpha_fo, turn.Nfo, 0, Front));
% 
%         % Force Components
%         Fp_fix = -Fp_fi * cos(delta_fi);
%         Fp_fiy = -Fp_fi * sin(delta_fi);
%         Fp_fox = -Fp_fo * cos(delta_fo);
%         Fp_foy = -Fp_fo * sin(delta_fo);
%         Fp_rix = -Fp_ri;
%         Fp_riy = 0;
%         Fp_rox = -Fp_ro;
%         Fp_roy = 0;
% 
%         % Centripetal Components
%         Fc_fi = abs(Fp_fi * cos(delta_fi - theta));
%         Fc_fo = abs(Fp_fo * cos(delta_fo - theta));
%         Fc_ri = abs(Fp_ri * cos(theta));
%         Fc_ro = abs(Fp_ro * cos(theta)); 
% 
%         % Residuals
%         eq1 = ...
%             M(r_fix, r_fiy, Fp_fix, Fp_fiy)+...
%             M(r_fox, r_foy, Fp_fox, Fp_foy)+...
%             M(r_rix, r_riy, Fp_rix, Fp_riy)+...
%             M(r_rox, r_roy, Fp_rox, Fp_roy);
%         eq2 = ...
%             Fc_fi + Fc_fo + Fc_ri + Fc_ro ...
%             - car.m * a_net;
% 
%         F = [eq1
%              eq2];
%     end
% 
% 
% end




























































function [sol, ok] = SolveEq(car, R, theta, Front, Rear, x_prev)

%-------------------------------------------------------------
% For ONE IC position (theta) on a corner of radius R, finds the
% MAXIMUM speed at which the car can be in equilibrium.
%
% Unknowns  x = [v (m/s); alpha_fi (rad); alpha_fo (rad)]
% Maximise  v
% Subject to yaw-moment balance = 0
%            centripetal-force balance = 0
%            0 <= alpha_fi, alpha_fo <= Front.alpha_limit
%
% NOTE: v is NOT a free "the lower the easier" variable. The rear
% slip angles are fixed by theta, so the rear tyres always push
% with a definite force, and that force can only be balanced at
% a definite range of speeds. Speed therefore has to be solved
% for, not searched upward from a starting value.
%
% INPUT
%   x_prev : (optional) solution from the neighbouring theta, [] if none
%
% OUTPUT
%   sol.v, sol.alpha_fi, sol.alpha_fo, sol.info
%   ok = true if an equilibrium point exists for this theta
%-------------------------------------------------------------

if nargin < 6
    x_prev = [];
end

sol = struct('v',NaN, 'alpha_fi',NaN, 'alpha_fo',NaN, 'info',[]);
ok  = false;


%% Rear slip limit (geometry only)
[~, g] = Balance(car, R, theta, 1, 0, 0, Front, Rear);

if max(abs([g.alpha_ri, g.alpha_ro])) > Rear.alpha_limit
    return
end


%% Bounds
lb = [0.5;  0;                 0];
ub = [80;   Front.alpha_limit; Front.alpha_limit];


%% Starting points (warm start + two cold guesses)
vGuess = sqrt(1.2*car.g*R);
a0     = Front.alpha_limit/2;

S = [vGuess,   0.5*vGuess;
     a0,       a0;
     a0,       a0];

if ~isempty(x_prev)
    S = [x_prev(:), S];
end


%% Solve
options = optimoptions('fmincon',...
                       'Algorithm','sqp',...
                       'Display','off',...
                       'ConstraintTolerance',1e-6,...      % residuals are in kN / kN*m
                       'OptimalityTolerance',1e-8,...
                       'StepTolerance',1e-10,...
                       'MaxIterations',300);

bestV = -inf;

for k = 1:size(S,2)

    [xs, ~, exitflag] = fmincon(@(x) -x(1), S(:,k),...
                                [], [], [], [], lb, ub,...
                                @nonlcon, options);

    if exitflag <= 0
        continue
    end

    % Forces must actually balance (0.05 N / N*m)
    [~, ceq] = nonlcon(xs);
    if norm(ceq)*1000 > 0.05
        continue
    end

    if xs(1) > bestV
        bestV        = xs(1);
        sol.v        = xs(1);
        sol.alpha_fi = xs(2);
        sol.alpha_fo = xs(3);
    end

end

if isfinite(bestV)
    [~, sol.info] = Balance(car, R, theta, sol.v, sol.alpha_fi, sol.alpha_fo, Front, Rear);
    ok = true;
end


    function [c, ceq] = nonlcon(x)
        c   = [];
        ceq = Balance(car, R, theta, x(1), x(2), x(3), Front, Rear)/1000;
    end

end