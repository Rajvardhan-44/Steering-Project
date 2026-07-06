function Fy = Pacejka96(alpha,Fz,gamma,tyre)

%-------------------------------------------------------------
% Pacejka '96 Pure Lateral Force Model
%
% INPUTS
% alpha : Slip angle (rad, magnitude)
% Fz    : Vertical load (N)
% gamma : Camber angle (rad)
%
% OUTPUT
% Fy    : Lateral force magnitude (N)
%-------------------------------------------------------------

alpha = abs(alpha);
gamma = abs(gamma);

%% Normalized load
dfz = (Fz-tyre.Fz0)/tyre.Fz0;

%% Scaling
muy = 1;

%% Shape
Cy = abs(tyre.pCY1);

%% Peak
Dy = abs(Fz*(tyre.pDY1 + tyre.pDY2*dfz) ...
    *(1-tyre.pDY3*gamma^2)*muy);

%% Cornering stiffness
Ky = abs(tyre.pKY1*tyre.Fz0 ...
    *sin(2*atan(Fz/(tyre.pKY2*tyre.Fz0))) ...
    *(1-tyre.pKY3*gamma));

%% Stiffness
By = Ky/(Cy*Dy);

%% Horizontal shift
SHy = 0;

alphay = alpha + SHy;

%% Curvature
Ey = abs(tyre.pEY1 + tyre.pEY2*dfz);

%% Vertical shift
SVy = 0;

%% Magic Formula
Fy = Dy*sin( Cy*atan( By*alphay ...
    - Ey*(By*alphay-atan(By*alphay)) ) ) + SVy;

Fy = abs(Fy);

end