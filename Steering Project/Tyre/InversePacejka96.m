function alpha = InversePacejka96(Fy,Fz,gamma,tyre)

%-------------------------------------------------------------
% Inverse Pacejka '96
%
% INPUT
% Fy : Lateral force magnitude (N)
%
% OUTPUT
% alpha : Slip angle magnitude (deg)
%-------------------------------------------------------------

Fy = abs(Fy);

alpha_limit = tyre.alpha_limit;

Fy_max = Pacejka96(alpha_limit,Fz,gamma,tyre);

assert(Fy<=Fy_max+1e-6,...
    'Requested lateral force exceeds tyre capacity.');

fun = @(a) Pacejka96(a,Fz,gamma,tyre)-Fy;

alpha = fzero(fun,[0 alpha_limit]);

alpha = rad2deg(alpha);

end