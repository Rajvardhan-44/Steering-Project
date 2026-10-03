function res = MechanismResidualsJoint(z, a, r, targetInner, targetOuter)
%-------------------------------------------------------------
% Joint residual for lsqnonlin.
%
% Unlike MechanismResiduals.m (which forces the inner angle to match
% exactly via a root-find, dumping all remaining error onto the
% outer angle), this lets the rack displacement AT EACH RADIUS be its
% own free variable, alongside the shared mechanism parameters. The
% optimizer then naturally splits the error between inner and outer
% however minimizes the total - usually a much more balanced fit.
%
% z = [d, t, s, toe0, x_1, x_2, ..., x_n]
%       first 4 entries: shared mechanism parameters
%       remaining n entries: rack displacement for radius i
%-------------------------------------------------------------

p = z(1:4);
x = z(5:end);
x = x(:);

[achInner, achOuter] = MechanismDeltas(p, a, r, x);

resInner = achInner(:) - targetInner(:);
resOuter = achOuter(:) - targetOuter(:);

res = [resInner; resOuter];

end