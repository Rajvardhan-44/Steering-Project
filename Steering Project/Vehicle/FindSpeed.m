function Result = FindSpeed(car, Front, Rear)

%---------------------------------------------------------
% Finds maximum cornering speed for different corner radii
%---------------------------------------------------------

% Corner Radius (m)
Radius = car.radius;

% Setup

n = length(Radius);

Result.Radius = Radius;

Result.Speed   = zeros(n,1);
Result.accel   = zeros(n,1);

Result.alphaFO = zeros(n,1);
Result.alphaFI = zeros(n,1);

Result.alphaRO = zeros(n,1);
Result.alphaRI = zeros(n,1);

%% Initial Speed (kmph)
SpeedStart = 1;

%% Main Loop

for i = 1:n

    R = Radius(i);

    speed = SpeedStart;

    FineSearch = false;

    while true

        v = speed/3.6;              % Vehicle speed (m/s)

        move = Moving(car,v);       % Aero loads

        accel = v^2/R;              % Lateral acceleration

        %% Tyre loads and forces
        turn = Turn(car,move,accel,Front,Rear);

        %% Has tyre limit been crossed?

        if (turn.Fyf > turn.FyMaxFront) || (turn.Fyr > turn.FyMaxRear)

            if ~FineSearch

                % Go back to last unchecked speed
                speed = speed - 4;
                FineSearch = true;

            else

                % Previous speed was the maximum safe speed
                speed = speed - 1;
                break

            end

        else

            if FineSearch
                speed = speed + 1;
            else
                speed = speed + 5;
            end

        end

    end

    %% Final calculation at Vmax

    v = speed/3.6;

    move = Moving(car,v);

    accel = v^2/R;

    turn = Turn(car,move,accel,Front,Rear);


























    fprintf('\n=====================================\n');
    fprintf('Radius = %.1f m\n',R);
    fprintf('Speed  = %.1f km/h\n',speed);

    fprintf('Nfo = %.2f N\n',turn.Nfo);
    fprintf('Nfi = %.2f N\n',turn.Nfi);
    fprintf('Nro = %.2f N\n',turn.Nro);
    fprintf('Nri = %.2f N\n',turn.Nri);
    fprintf('=====================================\n');























    
    slip = FindSlip(turn,Front,Rear);






    fprintf('\n');
    fprintf('alphaRI = %.4f deg\n', slip.alphaRI);
    fprintf('alphaRO = %.4f deg\n', slip.alphaRO);
    fprintf('Difference = %.6f deg\n', slip.alphaRO-slip.alphaRI);


























    %% Store Results

    Result.Speed(i) = speed;
    Result.accel(i) = accel;

    Result.alphaFO(i) = slip.alphaFO;
    Result.alphaFI(i) = slip.alphaFI;

    Result.alphaRO(i) = slip.alphaRO;
    Result.alphaRI(i) = slip.alphaRI;

    % figure
    % plot(Result.alphaRI, Result.alphaRI, 'r--','LineWidth',2)
    % hold on
    % plot(Result.alphaRI, Result.alphaRO, 'w--','LineWidth',2)

    %% Next corner starts from previous Vmax

    SpeedStart = speed;

end

end