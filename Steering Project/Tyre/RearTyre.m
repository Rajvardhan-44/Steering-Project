function tyre = RearTyre()

tyre.Fz0 = 3850;

tyre.pCY1 = 0.558238;

tyre.pDY1 = -2.23053;
tyre.pDY2 = 0.090785;
tyre.pDY3 = -5.71836;

tyre.pEY1 = -0.40009;
tyre.pEY2 = 0.569694;
tyre.pEY3 = -0.26276;
tyre.pEY4 = -29.3487;

tyre.pKY1 = -28.2448;
tyre.pKY2 = 1.331304;
tyre.pKY3 = 0.255683;

tyre.pHY1 = 0.00847;
tyre.pHY2 = 0.000594;
tyre.pHY3 = 0.042;

tyre.pVY1 = 0.0262;
tyre.pVY2 = -0.0791;
tyre.pVY3 = -0.08552;
tyre.pVY4 = -0.44481;

%% Model Limits
tyre.alpha_limit = deg2rad(10);

% Because from this model we are not achieving maxima in Lateral force vs slip angle

end