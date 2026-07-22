function Limit = SolveEq(car,...
                         v,...
                         a_net,...
                         a_lat,...
                         a_long,...
                         theta,...
                         vel_fi,...
                         vel_fo,...
                         alpha_ri,...
                         alpha_ro,...
                         Front,...
                         Rear)


%-------------------------------------------------------------
% Checks whether a given speed is physically possible.
%
% OUTPUT
%   Limit = true   -> Car can sustain this speed within tire limits
%   Limit = false  -> Impossible (or exceeds 10°/tire limits)
%-------------------------------------------------------------



% Checking Rear Slip angles are in limit or not
if abs(alpha_ri) > Rear.alpha_limit || abs(alpha_ro) > Rear.alpha_limit
    Limit = false;
    return
end


% Moment function
M = @(rx,ry,Fx,Fy) rx.*Fy - ry.*Fx;


% Vehicle Loads
move = Moving(car, v);
turn = Turn(car, move, a_long, a_lat);


% Wheel Locations
r_fix = -car.t/2;
r_fiy = (1-car.wd)*car.l;
r_fox =  car.t/2;
r_foy = (1-car.wd)*car.l;
r_rix = -car.t/2;
r_riy = -car.wd*car.l;
r_rox =  car.t/2;
r_roy = -car.wd*car.l;


% Rear Tyre Forces
Fp_ri = abs(Pacejka96(alpha_ri, turn.Nri, 0, Rear));
Fp_ro = abs(Pacejka96(alpha_ro, turn.Nro, 0, Rear));


% Unknown front slip angles solver
fun = @(x) Residual(x);

% Starts the guess in the middle of the safe zone
x0 = [Front.alpha_limit/2
      Front.alpha_limit/2];

% Prevents solver from ever exceeding the front tire limit
lb = [0; 
      0];

ub = [Front.alpha_limit; 
      Front.alpha_limit];

options = optimoptions('lsqnonlin',...
                       'Display','off',...
                       'FunctionTolerance',1e-8,...
                       'StepTolerance',1e-8);


[~, ~, residual, exitflag] = lsqnonlin(fun, x0, lb, ub, options);


% Check Solver Success & Convergence Sanity
% exitflag <= 0 means solver failed.
% norm(residual) > 1e-3 ensures the mathematical forces actually balance.
if exitflag <= 0 || norm(residual) > 1e-3
    fprintf('SolveEq failed: v = %.2f km/h, exitflag = %d, residual = %.6f\n',...
        v*3.6, exitflag, norm(residual));
    Limit = false;
    return
end

Limit = true;


    function F = Residual(x)
        alpha_fi = x(1);
        alpha_fo = x(2);
        
        % Steering Angles
        delta_fi = vel_fi + alpha_fi;
        delta_fo = vel_fo + alpha_fo;
        
        % Front Pacejka Forces
        Fp_fi = abs(Pacejka96(alpha_fi, turn.Nfi, 0, Front));
        Fp_fo = abs(Pacejka96(alpha_fo, turn.Nfo, 0, Front));
        
        % Force Components
        Fp_fix = -Fp_fi * cos(delta_fi);
        Fp_fiy = -Fp_fi * sin(delta_fi);
        Fp_fox = -Fp_fo * cos(delta_fo);
        Fp_foy = -Fp_fo * sin(delta_fo);
        Fp_rix = -Fp_ri;
        Fp_riy = 0;
        Fp_rox = -Fp_ro;
        Fp_roy = 0;
        
        % Centripetal Components
        Fc_fi = abs(Fp_fi * cos(delta_fi - theta));
        Fc_fo = abs(Fp_fo * cos(delta_fo - theta));
        Fc_ri = abs(Fp_ri * cos(theta));
        Fc_ro = abs(Fp_ro * cos(theta)); 
        
        % Residuals
        eq1 = ...
            M(r_fix, r_fiy, Fp_fix, Fp_fiy)+...
            M(r_fox, r_foy, Fp_fox, Fp_foy)+...
            M(r_rix, r_riy, Fp_rix, Fp_riy)+...
            M(r_rox, r_roy, Fp_rox, Fp_roy);
        eq2 = ...
            Fc_fi + Fc_fo + Fc_ri + Fc_ro ...
            - car.m * a_net;
            
        F = [eq1
             eq2];
    end


end