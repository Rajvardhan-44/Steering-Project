% function [alphaL, alphaR] = steeringAngles(x,a,r,s,d,t)
% 
% % x : rack displacement to right side
% % a : distance between steering-arm fixed pivots
% % r : rack length
% % s : steering-arm length
% % d : distance between rack and front axle
% % t : tie-rod length
% %
% % alphaL, alphaR : steering-arm angles in degrees
% 
% 
% 
% % --------------- right side ---------------
% C_R = (a-r)/2 - x;
% 
% K_R = (C_R.^2 + s^2 + d^2 - t^2) ./ ...
%     (2*s*sqrt(C_R.^2 + d^2));
% 
% % Avoid tiny numerical errors such as 1.00000000001
% K_R = min(max(K_R,-1),1);
% 
% phi_R = atan2d(d,C_R);
% 
% % Two mathematical solutions
% alphaR_1 = asind(K_R) - phi_R;
% alphaR_2 = 180 - asind(K_R) - phi_R;
% 
% 
% % --------------- left side ---------------
% 
% C_L = (a-r)/2 + x;
% 
% K_L = (C_L.^2 + s^2 + d^2 - t^2) ./ ...
%     (2*s*sqrt(C_L.^2 + d^2));
% 
% K_L = min(max(K_L,-1),1);
% 
% phi_L = atan2d(d,C_L);
% 
% % Two mathematical solutions
% alphaL_1 = asind(K_L) - phi_L;
% alphaL_2 = 180 - asind(K_L) - phi_L;
% 
% 
% % Select the physically relevant solution
% % Your geometry has alpha between 0 and 90 degrees.
% 
% alphaR = alphaR_1;
% idxR = ~(alphaR_1 >= 0 & alphaR_1 <= 90);
% alphaR(idxR) = alphaR_2(idxR);
% 
% alphaL = alphaL_1;
% idxL = ~(alphaL_1 >= 0 & alphaL_1 <= 90);
% alphaL(idxL) = alphaL_2(idxL);
% 
% end










































function [alphaL, alphaR] = steeringAngles(x,a,r,s,d,t)

% x : rack displacement to right side
% a : distance between steering-arm fixed pivots
% r : rack length
% s : steering-arm length
% d : distance between rack and front axle
% t : tie-rod length
%
% alphaL, alphaR : steering-arm angles in degrees



% --------------- right side ---------------
C_R = (a-r)/2 - x;

K_R = (C_R.^2 + s^2 + d^2 - t^2) ./ ...
    (2*s*sqrt(C_R.^2 + d^2));

% Avoid tiny numerical errors such as 1.00000000001
K_R = min(max(K_R,-1),1);

phi_R = atan2d(d,C_R);

% Two mathematical solutions
alphaR_1 = asind(K_R) - phi_R;
alphaR_2 = 180 - asind(K_R) - phi_R;


% --------------- left side ---------------

C_L = (a-r)/2 + x;

K_L = (C_L.^2 + s^2 + d^2 - t^2) ./ ...
    (2*s*sqrt(C_L.^2 + d^2));

K_L = min(max(K_L,-1),1);

phi_L = atan2d(d,C_L);

% Two mathematical solutions
alphaL_1 = asind(K_L) - phi_L;
alphaL_2 = 180 - asind(K_L) - phi_L;


% Select the physically relevant solution
% Your geometry has alpha between 0 and 90 degrees.

alphaR = alphaR_1;
idxR = ~(alphaR_1 >= 0 & alphaR_1 <= 90);
alphaR(idxR) = alphaR_2(idxR);

alphaL = alphaL_1;
idxL = ~(alphaL_1 >= 0 & alphaL_1 <= 90);
alphaL(idxL) = alphaL_2(idxL);

end