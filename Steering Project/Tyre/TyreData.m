function [alpha,Fy] = TyreData(tyre,Fz,gamma)

%-------------------------------------------------------------
% Tyre curve
%-------------------------------------------------------------

alpha = deg2rad(0:0.1:15);

Fy = zeros(size(alpha));

for i=1:length(alpha)

    Fy(i)=Pacejka96(alpha(i),Fz,gamma,tyre);

end

end