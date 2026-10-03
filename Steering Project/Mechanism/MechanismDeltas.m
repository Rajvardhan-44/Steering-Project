function [deltaInner, deltaOuter, deltaL, deltaR] = MechanismDeltas(p, a, r, x)
%-------------------------------------------------------------
% Converts raw steering-arm angles (from steeringAngles.m) into
% actual wheel steer angles, and sorts them into inner/outer.
%
% INPUTS
% p : [d, t, s, toe0]
%       d    - rack's position from front axle (m)
%       t    - tie-rod length (m)
%       s    - steering-arm length (m)
%       toe0 - net static offset, deg (combines "steering arm
%              mounting angle" and "toe at zero steer" - these are
%              mathematically identical in this planar model, both
%              being a constant additive shift applied equally to
%              both wheels at the zero-rack-displacement condition)
% a : distance between steering-arm fixed pivots (m) - fixed, from
%     suspension geometry
% r : rack length (m) - fixed, from inner pivot spacing
% x : rack displacement (m), scalar or vector, x >= 0 by convention
%     (mirror symmetry handles the opposite lock direction)
%
% OUTPUTS
% deltaInner, deltaOuter : wheel steer angle magnitudes (deg)
% deltaL, deltaR         : signed left/right wheel steer angles (deg),
%                          relative to straight-ahead, for reference
%-------------------------------------------------------------

d    = p(1);
t    = p(2);
s    = p(3);
toe0 = p(4);

% Baseline (zero rack displacement) arm angles - this is what
% "straight ahead" corresponds to for this geometry.
[alphaL0, alphaR0] = steeringAngles(0, a, r, s, d, t);

% Arm angles at the requested rack displacement(s)
[alphaL, alphaR] = steeringAngles(x, a, r, s, d, t);

deltaL = (alphaL - alphaL0) + toe0;
deltaR = (alphaR - alphaR0) + toe0;

% Whichever side has the larger magnitude is the inner wheel for
% this rack direction; robust to which physical side that is.
deltaInner = max(abs(deltaL), abs(deltaR));
deltaOuter = min(abs(deltaL), abs(deltaR));

end