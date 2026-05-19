function g = constraint_takeoff(amphi_results, amphi_cfg)
% CONSTRAINT_TAKEOFF  Water takeoff distance constraint.
%   g = constraint_takeoff(amphi_results, amphi_cfg)
%
%   Ensures the water takeoff distance is within acceptable limits.
%
%   Constraint: takeoff_distance <= max_water_run  →  g <= 0
%
%   Inputs:
%     amphi_results - Struct from amphibious_analysis()
%     amphi_cfg     - Amphibious config struct
%
%   Output:
%     g - [1 x 3] Constraint values (g <= 0 means feasible)
%         g(1): Takeoff distance
%         g(2): Thrust margin at hump
%         g(3): Porpoising stability

    max_distance = amphi_cfg.takeoff.max_water_run;

    % g1: Takeoff distance constraint
    dist = amphi_results.takeoff.distance;
    g1 = (dist - max_distance) / max_distance;

    % g2: Thrust margin at hump (must be > 1.0)
    thrust_margin = amphi_results.takeoff.thrust_margin;
    g2 = (1.0 - thrust_margin);  % g <= 0 when margin >= 1

    % g3: Porpoising stability
    if amphi_results.hydro.porpoising_stable
        g3 = -1;  % Feasible (well within)
    else
        g3 = 1.0;  % Violated
    end

    g = [g1, g2, g3];

end
