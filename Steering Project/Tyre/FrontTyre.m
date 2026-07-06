function tyre = FrontTyre()

tyre.Fz0 = 2444;

tyre.pCY1 = 0.324013;

tyre.pDY1 = -3.674945;
tyre.pDY2 = 0.285134;
tyre.pDY3 = -2.494252;

tyre.pEY1 = -0.078785;
tyre.pEY2 = 0.245086;
tyre.pEY3 = -0.382274;
tyre.pEY4 = -6.25570332;

tyre.pKY1 = -41.7228113;
tyre.pKY2 = 2.11293838;
tyre.pKY3 = 0.150080764;

tyre.pHY1 = 0.00711;
tyre.pHY2 = -0.000509;
tyre.pHY3 = 0.049069131;

tyre.pVY1 = -0.00734;
tyre.pVY2 = -0.0778;
tyre.pVY3 = -0.0641;
tyre.pVY4 = -0.6978041;

%% Model Limits
tyre.alpha_limit = deg2rad(10);

% Because from this model we are not achieving maxima in Lateral force vs slip angle

end