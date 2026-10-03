function res = MechanismResiduals(p, a, r, targetInner, targetOuter)
%-------------------------------------------------------------
% Residual vector for lsqnonlin: for each target radius, solves for
% the rack displacement that matches the target inner angle, then
% compares both the achieved inner (should be ~0 unless saturated)
% and achieved outer angle against their targets.
%
% Returns a 2n x 1 vector (n inner residuals, then n outer residuals)
% so unreachable points still push the optimizer in a useful
% direction instead of just going silent.
%-------------------------------------------------------------

n = numel(targetInner);
resInner = zeros(n,1);
resOuter = zeros(n,1);

for i = 1:n
    [~, achInner, achOuter] = SolveRackForInner(targetInner(i), p, a, r);
    resInner(i) = achInner - targetInner(i);
    resOuter(i) = achOuter - targetOuter(i);
end

res = [resInner; resOuter];

end