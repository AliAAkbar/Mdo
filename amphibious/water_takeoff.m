function takeoff = water_takeoff(design_vars, prob_cfg, prop_cfg, amphi_cfg, aero_results, prop_results)
% WATER_TAKEOFF  Water takeoff distance and performance analysis.
%   takeoff = water_takeoff(design_vars, prob_cfg, prop_cfg, amphi_cfg, aero_results, prop_results)
%
%   Simulates the water takeoff run from rest to liftoff speed.
%   Accounts for hydrodynamic resistance, aerodynamic forces during
%   acceleration, and ground effect.
%
%   Inputs:
%     design_vars  - [1 x n_vars] Design variable vector
%     prob_cfg     - Problem config struct
%     prop_cfg     - Propulsion config struct
%     amphi_cfg    - Amphibious config struct
%     aero_results - Struct from compute_aero() (CL, CD, CL_max, etc.)
%     prop_results - Struct from propulsion_analysis() (thrust, power)
%
%   Output:
%     takeoff - Struct with fields:
%               .distance       - [m] Takeoff distance (water run)
%               .time           - [s] Time to liftoff
%               .V_liftoff      - [m/s] Liftoff speed
%               .V_stall        - [m/s] Stall speed
%               .hump_speed     - [m/s] Speed at hump resistance
%               .hump_resistance- [N] Peak water resistance (hump drag)
%               .thrust_margin  - [-] Thrust/Drag at hump (must be > 1)
%               .feasible       - Logical (distance < max allowed)
%               .speed_history  - [N x 2] (time, speed) trajectory
%               .force_history  - [N x 4] (V, Thrust, Drag_water, Drag_aero)

    % --- Map design variables ---
    var_names = prob_cfg.var_names;
    params = struct();
    for i = 1:numel(var_names)
        params.(var_names{i}) = design_vars(i);
    end

    % --- Aircraft parameters ---
    MTOW = prob_cfg.aircraft.MTOW;
    g = prob_cfg.atmo.g;
    W = MTOW * g;
    rho_air = prob_cfg.atmo.rho_sl;
    S_ref = aero_results.S_ref;

    % --- Takeoff speeds ---
    CL_max = aero_results.CL_max;
    V_stall = sqrt(2 * W / (rho_air * S_ref * CL_max));
    V_rotation = V_stall * amphi_cfg.takeoff.rotation_speed_factor;
    V_liftoff = V_rotation;  % Simplified: liftoff at rotation speed

    % Account for headwind
    V_ground_liftoff = V_liftoff - amphi_cfg.takeoff.wind_speed;

    % --- Available thrust (assume constant for simplification) ---
    T_available = prop_results.thrust_cruise;
    % At low speed, propeller produces more thrust (static thrust)
    % Use a simple model: T = T_cruise * (V_cruise/V)^0.3 for V > 5 m/s
    V_cruise = prob_cfg.mission.cruise_speed;

    % --- Numerical integration of takeoff run ---
    dt = 0.1;  % [s] Time step
    V = 0;     % [m/s] Initial speed
    x = 0;    % [m] Distance
    t = 0;    % [s] Time

    % Storage for history
    max_steps = 5000;
    speed_hist = zeros(max_steps, 2);
    force_hist = zeros(max_steps, 4);
    step = 0;

    hump_resistance = 0;
    hump_speed = 0;

    while V < V_ground_liftoff && x < amphi_cfg.takeoff.max_water_run * 2 && t < 300
        step = step + 1;

        % Thrust at current speed
        if V < 5
            T = T_available * 2.5;  % Static thrust multiplier
        else
            T = T_available * (V_cruise / V)^0.3;
            T = min(T, T_available * 2.5);  % Cap
        end

        % Hydrodynamic resistance
        hydro = hull_hydro(max(V, 0.1), params, prob_cfg, amphi_cfg);
        R_water = hydro.resistance_total;

        % Track hump
        if R_water > hump_resistance
            hump_resistance = R_water;
            hump_speed = V;
        end

        % Aerodynamic forces during takeoff run
        q = 0.5 * rho_air * V^2;
        % Use reduced CL during ground run (no rotation yet)
        CL_ground = 0.3 * CL_max;  % Partial lift on water
        L_aero = q * S_ref * CL_ground;

        % Aerodynamic drag (reduced by ground effect)
        CD_ground = aero_results.CD * amphi_cfg.takeoff.ground_effect_factor;
        D_aero = q * S_ref * CD_ground;

        % Net force
        % Weight supported by water decreases as aero lift increases
        W_on_water = max(0, W - L_aero);
        % Water resistance scales with load on water
        load_fraction = W_on_water / W;
        R_water_actual = R_water * load_fraction;

        % Wave penalty
        if amphi_cfg.takeoff.wave_height > 0.1
            wave_penalty = 1 + 0.5 * amphi_cfg.takeoff.wave_height;
            R_water_actual = R_water_actual * wave_penalty;
        end

        F_net = T - R_water_actual - D_aero;

        % Acceleration
        a = F_net / MTOW;  % F = ma (mass constant for electric)
        a = max(a, -5);    % Limit deceleration (physical)

        % Update state
        V = V + a * dt;
        V = max(0, V);  % Can't go backwards
        x = x + V * dt;
        t = t + dt;

        % Store history
        if step <= max_steps
            speed_hist(step, :) = [t, V];
            force_hist(step, :) = [V, T, R_water_actual, D_aero];
        end

        % Safety: if can't accelerate past hump, stop
        if V < 1 && t > 30
            break;
        end
    end

    % Trim history
    speed_hist = speed_hist(1:min(step, max_steps), :);
    force_hist = force_hist(1:min(step, max_steps), :);

    % --- Feasibility ---
    distance_feasible = (x <= amphi_cfg.takeoff.max_water_run);
    thrust_margin = T_available * 2.5 / max(hump_resistance, 1);
    hump_feasible = (thrust_margin > 1.0);
    feasible = distance_feasible && hump_feasible && (V >= V_ground_liftoff * 0.95);

    % --- Build output struct ---
    takeoff = struct();
    takeoff.distance        = x;
    takeoff.time            = t;
    takeoff.V_liftoff       = V_liftoff;
    takeoff.V_stall         = V_stall;
    takeoff.hump_speed      = hump_speed;
    takeoff.hump_resistance = hump_resistance;
    takeoff.thrust_margin   = thrust_margin;
    takeoff.feasible        = feasible;
    takeoff.speed_history   = speed_hist;
    takeoff.force_history   = force_hist;

end
