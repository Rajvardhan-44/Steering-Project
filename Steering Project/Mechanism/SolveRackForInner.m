function [x_sol, achievedInner, achievedOuter, reachable] = ...
    SolveRackForInner(targetInner, p, a, r)
%-------------------------------------------------------------
% Finds the rack displacement x (>=0) such that the mechanism's
% inner steer angle equals targetInner (deg). Inner angle is assumed
% monotonically increasing in x over the physical range, which holds
% for a sane steering geometry.
%
% If the target can't be reached (mechanism saturates before getting
% there - the linkage equivalent of hitting a rack stop), returns the
% best achievable point instead of erroring, with reachable = false.
%-------------------------------------------------------------

innerAt = @(x) MechanismDeltas(p, a, r, x);

xLow  = 0;
gLow  = innerAt(xLow) - targetInner;

xHigh = 0.02;           % initial bracket guess (m), expanded below
maxExpand = 40;
gHigh = innerAt(xHigh) - targetInner;

count = 0;
while sign(gLow) == sign(gHigh) && count < maxExpand
    xHigh = xHigh * 1.5;
    gHigh = innerAt(xHigh) - targetInner;
    count = count + 1;
end

if sign(gLow) == sign(gHigh)
    % Could not bracket a root within a reasonable rack travel -
    % target is not achievable with this parameter set. Report the
    % best (furthest) point found so the residual still guides the
    % optimizer sensibly instead of throwing an error.
    x_sol = xHigh;
    reachable = false;
else
    g = @(x) innerAt(x) - targetInner;
    x_sol = fzero(g, [xLow, xHigh]);
    reachable = true;
end

[achievedInner, achievedOuter] = MechanismDeltas(p, a, r, x_sol);

end