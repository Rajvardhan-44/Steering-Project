function [delta_fi,delta_fo] = FindDelta(car,...
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
% Finds steering angles for the FINAL operating point.
%
% INPUT
%    Final IC
%    Final Speed
%
% OUTPUT
%    delta_fi
%    delta_fo
%-------------------------------------------------------------


if abs(alpha_ri) > Rear.alpha_limit || abs(alpha_ro) > Rear.alpha_limit
    error('FindDelta:RearSlipLimit', ...
        'Rear tire slip angles exceed limit (%.2f rad) before solving front angles.', Rear.alpha_limit);
end


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


% Rear Forces
Fp_ri = abs(Pacejka96(alpha_ri, turn.Nri, 0, Rear));
Fp_ro = abs(Pacejka96(alpha_ro, turn.Nro, 0, Rear));


% Moment Function
M = @(rx,ry,Fx,Fy) rx.*Fy-ry.*Fx;


% Solve for Front Slip Angles
fun = @(x) Residual(x);

x0 = [Front.alpha_limit/2
      Front.alpha_limit/2];

lb = [0
      0];

ub = [Front.alpha_limit
      Front.alpha_limit];

options = optimoptions('lsqnonlin',...
                       'Display','off',...
                       'FunctionTolerance',1e-9,...
                       'StepTolerance',1e-9);

[x, ~, residual, exitflag] = lsqnonlin(fun, x0, lb, ub, options);

if exitflag<=0 || norm(residual)>1e-3
    error('Unable to determine steering angles for this configuration.')
end

% Final Steering Angles
alpha_fi = x(1);
alpha_fo = x(2);

delta_fi = vel_fi + alpha_fi;
delta_fo = vel_fo + alpha_fo;

    function F = Residual(x)
        alpha_fi = x(1);
        alpha_fo = x(2);

        % Steering
        delta_fi_loc = vel_fi + alpha_fi;
        delta_fo_loc = vel_fo + alpha_fo;

        % Front Forces
        Fp_fi = abs(Pacejka96(alpha_fi, turn.Nfi, 0, Front));
        Fp_fo = abs(Pacejka96(alpha_fo, turn.Nfo, 0, Front));

        % Force Components
        Fp_fix = -Fp_fi * cos(delta_fi_loc);
        Fp_fiy = -Fp_fi * sin(delta_fi_loc);

        Fp_fox = -Fp_fo * cos(delta_fo_loc);
        Fp_foy = -Fp_fo * sin(delta_fo_loc);

        Fp_rix = -Fp_ri;
        Fp_riy = 0;

        Fp_rox = -Fp_ro;
        Fp_roy = 0;

        % Centripetal Components (Fixed: Removed abs from theta)
        Fc_fi = abs(Fp_fi * cos(delta_fi_loc - theta));
        Fc_fo = abs(Fp_fo * cos(delta_fo_loc - theta));
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