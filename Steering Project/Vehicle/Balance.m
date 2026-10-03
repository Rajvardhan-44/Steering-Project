% function [F, out] = Balance(car, R, theta, v, alpha_fi, alpha_fo, Front, Rear)
% 
% %-------------------------------------------------------------
% % Force / moment residuals for ONE candidate operating point.
% % (Replaces the duplicated Residual() code that lived inside
% %  SolveEq.m and FindDelta.m.)
% %
% % INPUT
% %   R         corner radius (m)
% %   theta     IC angle about the CG (rad)
% %   v         speed (m/s)
% %   alpha_fi  front-inner slip angle magnitude (rad)
% %   alpha_fo  front-outer slip angle magnitude (rad)
% %
% % OUTPUT
% %   F   = [yaw-moment residual (N*m); centripetal-force residual (N)]
% %   out = geometry + steering angles for this point
% %-------------------------------------------------------------
% 
% 
% %% IC geometry (y measured from the rear axle, IC on the inner/left side)
% ICx = -R*cos(theta);
% ICy =  car.b - R*sin(theta);
% 
% dxi = abs(ICx) - car.t/2;
% dxo = abs(ICx) + car.t/2;
% 
% % Rear slip angles are fixed by geometry alone
% alpha_ri = atan2(ICy, dxi);
% alpha_ro = atan2(ICy, dxo);
% 
% % Front wheel velocity directions
% vel_fi = atan2(car.l - ICy, dxi);
% vel_fo = atan2(car.l - ICy, dxo);
% 
% % Steering angles
% delta_fi = vel_fi + alpha_fi;
% delta_fo = vel_fo + alpha_fo;
% 
% 
% %% Accelerations and vertical loads
% a_net  = v^2/R;
% a_lat  = a_net*cos(theta);
% a_long = a_net*sin(theta);
% 
% move = Moving(car, v);
% turn = Turn(car, move, a_long, a_lat);
% 
% % Keep loads strictly positive (Pacejka96 gives 0/0 = NaN at Fz = 0)
% Nfi = max(turn.Nfi, 1);
% Nfo = max(turn.Nfo, 1);
% Nri = max(turn.Nri, 1);
% Nro = max(turn.Nro, 1);
% 
% 
% %% Tyre forces
% Fp_fi = Pacejka96(alpha_fi, Nfi, 0, Front);
% Fp_fo = Pacejka96(alpha_fo, Nfo, 0, Front);
% Fp_ri = Pacejka96(alpha_ri, Nri, 0, Rear);
% Fp_ro = Pacejka96(alpha_ro, Nro, 0, Rear);
% 
% 
% %% Wheel positions relative to the CG
% r_fix = -car.t/2;   r_fiy =  car.a;
% r_fox =  car.t/2;   r_foy =  car.a;
% r_rix = -car.t/2;   r_riy = -car.b;
% r_rox =  car.t/2;   r_roy = -car.b;
% 
% M = @(rx,ry,Fx,Fy) rx.*Fy - ry.*Fx;
% 
% 
% %% Force components (all pointing toward the IC)
% Fp_fix = -Fp_fi*cos(delta_fi);   Fp_fiy = -Fp_fi*sin(delta_fi);
% Fp_fox = -Fp_fo*cos(delta_fo);   Fp_foy = -Fp_fo*sin(delta_fo);
% Fp_rix = -Fp_ri;                 Fp_riy = 0;
% Fp_rox = -Fp_ro;                 Fp_roy = 0;
% 
% 
% %% Residuals
% eq1 = M(r_fix, r_fiy, Fp_fix, Fp_fiy) ...
%     + M(r_fox, r_foy, Fp_fox, Fp_foy) ...
%     + M(r_rix, r_riy, Fp_rix, Fp_riy) ...
%     + M(r_rox, r_roy, Fp_rox, Fp_roy);
% 
% Fc = Fp_fi*cos(delta_fi - theta) ...
%    + Fp_fo*cos(delta_fo - theta) ...
%    + Fp_ri*cos(theta) ...
%    + Fp_ro*cos(theta);
% 
% eq2 = Fc - car.m*a_net;
% 
% F = [eq1; eq2];
% 
% 
% %% Extra info
% out.ICx      = ICx;
% out.ICy      = ICy;
% out.alpha_ri = alpha_ri;
% out.alpha_ro = alpha_ro;
% out.delta_fi = delta_fi;
% out.delta_fo = delta_fo;
% 
% end





















function [F, out] = Balance(car, R, theta, v, alpha_fi, alpha_fo, Front, Rear)
%-------------------------------------------------------------
% Force / moment residuals for ONE candidate operating point.
% (Replaces the duplicated Residual() code that lived inside
%  SolveEq.m and FindDelta.m.)
%
% INPUT
%   R         corner radius (m)
%   theta     IC angle about the CG (rad)
%   v         speed (m/s)
%   alpha_fi  front-inner slip angle magnitude (rad)
%   alpha_fo  front-outer slip angle magnitude (rad)
%
% OUTPUT
%   F   = [yaw-moment residual (N*m); centripetal-force residual (N)]
%   out = geometry + steering angles for this point
%-------------------------------------------------------------


%% IC geometry (y measured from the rear axle, IC on the inner/left side)
ICx = -R*cos(theta);
ICy =  car.b - R*sin(theta);
dxi = abs(ICx) - car.t/2;
dxo = abs(ICx) + car.t/2;
% Rear slip angles are fixed by geometry alone
alpha_ri = atan2(ICy, dxi);
alpha_ro = atan2(ICy, dxo);
% Front wheel velocity directions
vel_fi = atan2(car.l - ICy, dxi);
vel_fo = atan2(car.l - ICy, dxo);
% Steering angles
delta_fi = vel_fi + alpha_fi;
delta_fo = vel_fo + alpha_fo;


%% Accelerations and vertical loads
a_net  = v^2/R;
a_lat  = a_net*cos(theta);
a_long = a_net*sin(theta);
move = Moving(car, v);
turn = Turn(car, move, a_long, a_lat);
% Keep loads strictly positive (Pacejka96 gives 0/0 = NaN at Fz = 0)
Nfi = max(turn.Nfi, 1);
Nfo = max(turn.Nfo, 1);
Nri = max(turn.Nri, 1);
Nro = max(turn.Nro, 1);
%% Tyre forces
Fp_fi = Pacejka96(alpha_fi, Nfi, 0, Front);
Fp_fo = Pacejka96(alpha_fo, Nfo, 0, Front);
Fp_ri = Pacejka96(alpha_ri, Nri, 0, Rear);
Fp_ro = Pacejka96(alpha_ro, Nro, 0, Rear);
%% Wheel positions relative to the CG
r_fix = -car.t/2;   r_fiy =  car.a;
r_fox =  car.t/2;   r_foy =  car.a;
r_rix = -car.t/2;   r_riy = -car.b;
r_rox =  car.t/2;   r_roy = -car.b;
M = @(rx,ry,Fx,Fy) rx.*Fy - ry.*Fx;
%% Force components
% Front: direction comes straight from the actual (signed) steer
% angle, so this was always correct regardless of IC position.
Fp_fix = -Fp_fi*cos(delta_fi);   Fp_fiy = -Fp_fi*sin(delta_fi);
Fp_fox = -Fp_fo*cos(delta_fo);   Fp_foy = -Fp_fo*sin(delta_fo);

% Rear: Pacejka96 returns a magnitude only (it takes abs(alpha)
% internally), so the direction has to be reconstructed here from the
% SIGNED slip angle. Previously this was hardcoded to always point
% toward the IC (-x), which is only correct while IC stays ahead of
% the rear axle (alpha_ri, alpha_ro >= 0 in that regime). Once the IC
% is allowed behind the rear axle, alpha_ri/alpha_ro can go negative
% and the true force direction reverses - using sign(alpha) here
% instead of a constant keeps this correct across the whole range,
% and is a no-op (identical results) everywhere the old assumption
% already held.
Fp_rix = -sign(alpha_ri)*Fp_ri;  Fp_riy = 0;
Fp_rox = -sign(alpha_ro)*Fp_ro;  Fp_roy = 0;
%% Residuals
eq1 = M(r_fix, r_fiy, Fp_fix, Fp_fiy) ...
    + M(r_fox, r_foy, Fp_fox, Fp_foy) ...
    + M(r_rix, r_riy, Fp_rix, Fp_riy) ...
    + M(r_rox, r_roy, Fp_rox, Fp_roy);
Fc = Fp_fi*cos(delta_fi - theta) ...
    + Fp_fo*cos(delta_fo - theta) ...
    + sign(alpha_ri)*Fp_ri*cos(theta) ...
    + sign(alpha_ro)*Fp_ro*cos(theta);
eq2 = Fc - car.m*a_net;
F = [eq1; eq2];
%% Extra info
out.ICx      = ICx;
out.ICy      = ICy;
out.alpha_ri = alpha_ri;
out.alpha_ro = alpha_ro;
out.delta_fi = delta_fi;
out.delta_fo = delta_fo;
end