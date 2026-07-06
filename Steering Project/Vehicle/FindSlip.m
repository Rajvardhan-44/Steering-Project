function slip = FindSlip(turn, Front, Rear)

%-------------------------------------------------------------
% Find slip angles corresponding to required tyre forces
%-------------------------------------------------------------

gamma = 0;      % Camber

%% Front Outer

slip.alphaFO = InversePacejka96(turn.Fyfo, turn.Nfo, gamma, Front);

%% Front Inner

slip.alphaFI = InversePacejka96(turn.Fyfi, turn.Nfi, gamma, Front);

%% Rear Outer

slip.alphaRO = InversePacejka96(turn.Fyro, turn.Nro, gamma, Rear);

%% Rear Inner

slip.alphaRI = InversePacejka96(turn.Fyri, turn.Nri, gamma, Rear);

end