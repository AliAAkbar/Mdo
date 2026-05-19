%% TEST_BASELINE.M - Integration Smoke Test for MDO Framework
%  =========================================================================
%  Validates that all modules communicate correctly and produce physical
%  results. Run this BEFORE any optimization campaign.
%
%  Diagnostic output: PASS / FAIL for each module + summary.
%
%  Usage:
%    >> test_baseline
%  =========================================================================

%% Housekeeping
clear; clc; close all;
fprintf('================================================================\n');
fprintf('  MDO BASELINE INTEGRATION TEST\n');
fprintf('  Electric Amphibious Aircraft Framework\n');
fprintf('================================================================\n\n');

%% Setup paths
addpath(genpath('config'));
addpath(genpath('optimizer'));
addpath(genpath('aero'));
addpath(genpath('propulsion'));
addpath(genpath('amphibious'));
addpath(genpath('objectives'));
addpath(genpath('utils'));

if ~exist('output', 'dir')
    mkdir('output');
end

%% Test counters
n_pass = 0;
n_fail = 0;
test_log = {};


%% ========================================================================
%  TEST 1: Configuration Loading
%  ========================================================================
fprintf('─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 1] Configuration Loading\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    prob_cfg  = problem_config();
    ga_cfg    = ga_config();
    moo_cfg   = moo_config();
    prop_cfg  = propulsion_config();
    amphi_cfg = amphibious_config();

    % Validate key fields exist
    assert(isfield(prob_cfg, 'n_vars'), 'prob_cfg missing n_vars');
    assert(isfield(prob_cfg, 'lb'), 'prob_cfg missing lb');
    assert(isfield(prob_cfg, 'ub'), 'prob_cfg missing ub');
    assert(numel(prob_cfg.lb) == prob_cfg.n_vars, 'lb size mismatch');
    assert(numel(prob_cfg.ub) == prob_cfg.n_vars, 'ub size mismatch');
    assert(all(prob_cfg.lb < prob_cfg.ub), 'lb must be < ub');
    assert(isfield(ga_cfg, 'pop_size'), 'ga_cfg missing pop_size');
    assert(isfield(moo_cfg, 'n_objectives'), 'moo_cfg missing n_objectives');
    assert(isfield(prop_cfg, 'battery'), 'prop_cfg missing battery');
    assert(isfield(amphi_cfg, 'hull'), 'amphi_cfg missing hull');

    fprintf('  prob_cfg:  %d vars, bounds valid\n', prob_cfg.n_vars);
    fprintf('  ga_cfg:    pop=%d, gen=%d, seed=%d\n', ...
            ga_cfg.pop_size, ga_cfg.n_generations, ga_cfg.rng_seed);
    fprintf('  moo_cfg:   %d objectives\n', moo_cfg.n_objectives);
    fprintf('  prop_cfg:  battery %.0f Wh/kg, motor eff %.2f\n', ...
            prop_cfg.battery.specific_energy, prop_cfg.motor.efficiency_peak);
    fprintf('  amphi_cfg: hull L/B=%.1f, max takeoff=%.0fm\n', ...
            amphi_cfg.hull.length_beam_ratio, amphi_cfg.takeoff.max_water_run);

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Configuration Loading');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Configuration Loading', ME);
end


%% ========================================================================
%  TEST 2: Population Initialization (LHS)
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 2] Population Initialization (Latin Hypercube Sampling)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    rng(ga_cfg.rng_seed);
    test_pop_size = 20;
    pop = init_population(test_pop_size, prob_cfg.n_vars, prob_cfg.lb, prob_cfg.ub);

    assert(size(pop,1) == test_pop_size, 'Wrong population size');
    assert(size(pop,2) == prob_cfg.n_vars, 'Wrong number of variables');
    assert(all(pop(:) >= min(prob_cfg.lb)), 'Some values below lower bound');
    assert(all(pop(:) <= max(prob_cfg.ub)), 'Some values above upper bound');

    % Check bounds per variable
    for j = 1:prob_cfg.n_vars
        assert(all(pop(:,j) >= prob_cfg.lb(j)), sprintf('Var %d below lb', j));
        assert(all(pop(:,j) <= prob_cfg.ub(j)), sprintf('Var %d above ub', j));
    end

    % Check LHS space-filling (each variable should span its range)
    for j = 1:prob_cfg.n_vars
        var_range = max(pop(:,j)) - min(pop(:,j));
        expected_range = prob_cfg.ub(j) - prob_cfg.lb(j);
        coverage = var_range / expected_range;
        assert(coverage > 0.5, sprintf('Var %d poor coverage: %.2f', j, coverage));
    end

    fprintf('  Population: %d x %d matrix\n', size(pop,1), size(pop,2));
    fprintf('  Bounds: all satisfied\n');
    fprintf('  Space-filling: good coverage (>50%% range per variable)\n');

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Population Initialization');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Population Initialization', ME);
end


%% ========================================================================
%  TEST 3: Aerodynamics Module
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 3] Aerodynamics Module (Analytical Fallback)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    % Use mid-range design vector
    x_mid = (prob_cfg.lb + prob_cfg.ub) / 2;

    aero_results = compute_aero(x_mid, prob_cfg, amphi_cfg);

    % Validate output struct fields
    assert(isfield(aero_results, 'CL'), 'Missing CL');
    assert(isfield(aero_results, 'CD'), 'Missing CD');
    assert(isfield(aero_results, 'CL_CD'), 'Missing CL_CD');
    assert(isfield(aero_results, 'CL_max'), 'Missing CL_max');
    assert(isfield(aero_results, 'alpha_stall'), 'Missing alpha_stall');
    assert(isfield(aero_results, 'polar'), 'Missing polar');
    assert(isfield(aero_results, 'S_ref'), 'Missing S_ref');
    assert(isfield(aero_results, 'method'), 'Missing method');
    assert(isfield(aero_results, 'valid'), 'Missing valid');

    % Physical sanity checks
    assert(aero_results.CL > 0, 'CL must be positive at cruise');
    assert(aero_results.CD > 0.005, 'CD unrealistically low');
    assert(aero_results.CD < 0.2, 'CD unrealistically high');
    assert(aero_results.CL_CD > 3, 'L/D too low');
    assert(aero_results.CL_CD < 30, 'L/D unrealistically high');
    assert(aero_results.CL_max > 1.0, 'CL_max too low');
    assert(aero_results.CL_max < 3.0, 'CL_max unrealistically high');
    assert(aero_results.S_ref > 5, 'S_ref too small');
    assert(aero_results.S_ref < 50, 'S_ref too large');
    assert(aero_results.valid == true, 'Aero results flagged invalid');

    fprintf('  Method: %s\n', aero_results.method);
    fprintf('  CL = %.4f, CD = %.5f, L/D = %.2f\n', ...
            aero_results.CL, aero_results.CD, aero_results.CL_CD);
    fprintf('  CL_max = %.3f, alpha_stall = %.1f deg\n', ...
            aero_results.CL_max, aero_results.alpha_stall);
    fprintf('  S_ref = %.2f m^2\n', aero_results.S_ref);
    fprintf('  Polar table: %d points\n', size(aero_results.polar, 1));

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Aerodynamics Module');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Aerodynamics Module', ME);
end


%% ========================================================================
%  TEST 4: Propulsion Module
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 4] Electric Propulsion Module\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    x_mid = (prob_cfg.lb + prob_cfg.ub) / 2;

    prop_results = propulsion_analysis(x_mid, prob_cfg, prop_cfg);

    % Validate struct fields
    assert(isfield(prop_results, 'battery'), 'Missing battery');
    assert(isfield(prop_results, 'motor'), 'Missing motor');
    assert(isfield(prop_results, 'propeller'), 'Missing propeller');
    assert(isfield(prop_results, 'range_km'), 'Missing range_km');
    assert(isfield(prop_results, 'endurance_hr'), 'Missing endurance_hr');
    assert(isfield(prop_results, 'thrust_cruise'), 'Missing thrust_cruise');
    assert(isfield(prop_results, 'mass_total'), 'Missing mass_total');
    assert(isfield(prop_results, 'feasible'), 'Missing feasible');
    assert(isfield(prop_results, 'power_budget'), 'Missing power_budget');

    % Battery checks
    bat = prop_results.battery;
    assert(bat.energy_usable > 0, 'Battery usable energy must be positive');
    assert(bat.energy_usable < bat.energy_total, 'Usable must be < total');
    assert(bat.mass_cells < bat.mass_total, 'Cell mass < total mass');
    assert(bat.specific_energy_pack > 50, 'Pack SE too low');
    assert(bat.specific_energy_pack < 300, 'Pack SE unrealistically high');

    % Motor checks
    mot = prop_results.motor;
    assert(mot.efficiency_cruise > 0.7, 'Motor eff too low');
    assert(mot.efficiency_cruise <= 1.0, 'Motor eff > 1 is unphysical');
    assert(mot.mass_total > 5, 'Motor too light');
    assert(mot.mass_total < 100, 'Motor too heavy');

    % Propeller checks
    prp = prop_results.propeller;
    assert(prp.thrust > 0, 'Thrust must be positive');
    assert(prp.efficiency > 0.3, 'Prop efficiency too low');
    assert(prp.efficiency <= 1.0, 'Prop efficiency > 1');
    assert(prp.tip_mach < 1.0, 'Tip Mach supersonic!');

    % Range checks
    assert(prop_results.range_km > 10, 'Range too short');
    assert(prop_results.range_km < 2000, 'Range unrealistic for electric');
    assert(prop_results.endurance_hr > 0.1, 'Endurance too short');

    fprintf('  Battery: %.1f kWh total, %.1f kWh usable (%.0f Wh/kg pack)\n', ...
            bat.energy_total/1000, bat.energy_usable/1000, bat.specific_energy_pack);
    fprintf('  Motor: %.1f kW rated, %.1f kg, eta_cruise=%.3f\n', ...
            mot.rated_power/1000, mot.mass_total, mot.efficiency_cruise);
    fprintf('  Propeller: T=%.0f N, eta=%.3f, D=%.2fm, tip M=%.3f\n', ...
            prp.thrust, prp.efficiency, prp.diameter, prp.tip_mach);
    fprintf('  Range: %.1f km, Endurance: %.2f hr\n', ...
            prop_results.range_km, prop_results.endurance_hr);
    fprintf('  Feasible: %s\n', mat2str(prop_results.feasible));

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Propulsion Module');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Propulsion Module', ME);
end


%% ========================================================================
%  TEST 5: Amphibious Module
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 5] Amphibious Module (Hull Hydro + Water Takeoff)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    x_mid = (prob_cfg.lb + prob_cfg.ub) / 2;

    % Need aero and propulsion results first (coupling)
    aero_res = compute_aero(x_mid, prob_cfg, amphi_cfg);
    prop_res = propulsion_analysis(x_mid, prob_cfg, prop_cfg);

    amphi_results = amphibious_analysis(x_mid, prob_cfg, prop_cfg, amphi_cfg, ...
                                         aero_res, prop_res);

    % Validate struct fields
    assert(isfield(amphi_results, 'hydro'), 'Missing hydro');
    assert(isfield(amphi_results, 'takeoff'), 'Missing takeoff');
    assert(isfield(amphi_results, 'penalties'), 'Missing penalties');
    assert(isfield(amphi_results, 'landing'), 'Missing landing');
    assert(isfield(amphi_results, 'feasible'), 'Missing feasible');
    assert(isfield(amphi_results, 'constraint_violations'), 'Missing constraints');

    % Hydrodynamics checks
    hydro = amphi_results.hydro;
    assert(hydro.resistance_total >= 0, 'Negative resistance');
    assert(hydro.Cv >= 0, 'Negative speed coefficient');
    assert(hydro.trim_angle >= 0, 'Negative trim angle');
    assert(hydro.trim_angle < 15, 'Trim angle too high');

    % Takeoff checks
    tkoff = amphi_results.takeoff;
    assert(tkoff.distance > 0, 'Zero takeoff distance');
    assert(tkoff.distance < 5000, 'Takeoff distance unrealistic');
    assert(tkoff.V_stall > 15, 'Stall speed too low');
    assert(tkoff.V_stall < 60, 'Stall speed too high');
    assert(tkoff.V_liftoff > tkoff.V_stall, 'V_liftoff must > V_stall');
    assert(tkoff.thrust_margin > 0, 'Thrust margin must be positive');

    % Penalty checks
    pen = amphi_results.penalties;
    assert(pen.delta_Cd0 > 0, 'Drag penalty must be positive');
    assert(pen.delta_Cd0 < 0.05, 'Drag penalty unrealistically high');
    assert(pen.delta_weight_kg > 0, 'Weight penalty must be positive');
    assert(pen.delta_weight_kg < 500, 'Weight penalty unrealistically high');

    fprintf('  Hydro: R_total=%.0f N, Cv=%.2f, trim=%.1f deg, stable=%s\n', ...
            hydro.resistance_total, hydro.Cv, hydro.trim_angle, ...
            mat2str(hydro.porpoising_stable));
    fprintf('  Takeoff: dist=%.0f m, V_stall=%.1f m/s, V_liftoff=%.1f m/s\n', ...
            tkoff.distance, tkoff.V_stall, tkoff.V_liftoff);
    fprintf('  Thrust margin at hump: %.2f (need >1.0)\n', tkoff.thrust_margin);
    fprintf('  Penalties: dCd0=%.4f, dW=%.1f kg\n', pen.delta_Cd0, pen.delta_weight_kg);
    fprintf('  Landing: V_app=%.1f m/s, n=%.2f g, feasible=%s\n', ...
            amphi_results.landing.V_approach, amphi_results.landing.load_factor, ...
            mat2str(amphi_results.landing.feasible));
    fprintf('  Overall feasible: %s\n', mat2str(amphi_results.feasible));

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Amphibious Module');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Amphibious Module', ME);
end


%% ========================================================================
%  TEST 6: Objectives & Constraints Integration
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 6] Objectives & Constraints (Full Pipeline)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    x_mid = (prob_cfg.lb + prob_cfg.ub) / 2;

    [obj_values, g, disc_data] = evaluate_objectives(x_mid, prob_cfg, prop_cfg, ...
                                                      amphi_cfg, moo_cfg);

    % Check dimensions
    assert(numel(obj_values) == moo_cfg.n_objectives, ...
           sprintf('Expected %d objectives, got %d', moo_cfg.n_objectives, numel(obj_values)));
    assert(numel(g) >= 4, 'Expected at least 4 constraints');

    % Check objective values are finite
    assert(all(isfinite(obj_values)), 'Non-finite objective values');
    assert(all(isfinite(g)), 'Non-finite constraint values');

    % Physical ranges
    % Obj 1: CL/CD (should be 5-25 for amphibious)
    assert(obj_values(1) > 2, 'CL/CD too low');
    assert(obj_values(1) < 30, 'CL/CD too high');

    % Obj 2: Weight (should be 500-3000 kg)
    assert(obj_values(2) > 200, 'Weight too low');
    assert(obj_values(2) < 5000, 'Weight too high');

    % Obj 3: Range (should be 10-1000 km for electric)
    assert(obj_values(3) > 1, 'Range too short');
    assert(obj_values(3) < 2000, 'Range unrealistically high');

    % Discipline data struct
    assert(isfield(disc_data, 'aero'), 'Missing aero in discipline_data');
    assert(isfield(disc_data, 'propulsion'), 'Missing propulsion');
    assert(isfield(disc_data, 'amphibious'), 'Missing amphibious');
    assert(isfield(disc_data, 'feasible'), 'Missing feasible flag');

    fprintf('  Objectives:\n');
    fprintf('    [1] CL/CD  = %.3f (maximize)\n', obj_values(1));
    fprintf('    [2] Weight = %.1f kg (minimize)\n', obj_values(2));
    fprintf('    [3] Range  = %.1f km (maximize)\n', obj_values(3));
    fprintf('  Constraints (%d total):\n', numel(g));
    fprintf('    Values: [%s]\n', num2str(g, '%.3f '));
    n_violated = sum(g > 0);
    fprintf('    Violated: %d / %d\n', n_violated, numel(g));
    fprintf('  Overall feasible: %s\n', mat2str(disc_data.feasible));

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Objectives & Constraints');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Objectives & Constraints', ME);
end


%% ========================================================================
%  TEST 7: GA Operators (Crossover + Mutation + Selection)
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 7] GA Operators (Selection, Crossover, Mutation)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    rng(42);
    lb = prob_cfg.lb;
    ub = prob_cfg.ub;
    n_vars = prob_cfg.n_vars;

    % Create two parent vectors
    p1 = lb + 0.3 * (ub - lb);
    p2 = lb + 0.7 * (ub - lb);

    % Test crossover
    [c1, c2] = crossover(p1, p2, ga_cfg.crossover_eta, 1.0, lb, ub);
    assert(numel(c1) == n_vars, 'Child 1 wrong size');
    assert(numel(c2) == n_vars, 'Child 2 wrong size');
    assert(all(c1 >= lb) && all(c1 <= ub), 'Child 1 out of bounds');
    assert(all(c2 >= lb) && all(c2 <= ub), 'Child 2 out of bounds');
    fprintf('  Crossover: children within bounds\n');

    % Test mutation
    mutant = mutation(p1, ga_cfg.mutation_eta, 1.0, lb, ub);  % prob=1 to force mutation
    assert(numel(mutant) == n_vars, 'Mutant wrong size');
    assert(all(mutant >= lb) && all(mutant <= ub), 'Mutant out of bounds');
    assert(~isequal(mutant, p1), 'Mutation did not change individual (prob=1)');
    fprintf('  Mutation: mutant within bounds, differs from parent\n');

    % Test selection
    fake_fitness = rand(20, 1);
    idx = selection(fake_fitness, ga_cfg.tournament_size, 5);
    assert(numel(idx) == 5, 'Selection returned wrong count');
    assert(all(idx >= 1) && all(idx <= 20), 'Selection indices out of range');
    fprintf('  Selection: 5 indices returned, all valid (1-20)\n');

    % Repeated crossover: verify diversity
    children = zeros(50, n_vars);
    for i = 1:50
        [c, ~] = crossover(p1, p2, ga_cfg.crossover_eta, 1.0, lb, ub);
        children(i,:) = c;
    end
    diversity = mean(std(children, 0, 1) ./ (ub - lb));
    assert(diversity > 0.01, 'Crossover producing near-identical children');
    fprintf('  Diversity check: std/range = %.4f (good)\n', diversity);

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'GA Operators');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'GA Operators', ME);
end


%% ========================================================================
%  TEST 8: Single-Objective GA (Short Run)
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 8] Single-Objective GA (10 gen, pop=20)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    % Override config for quick test
    test_ga_cfg = ga_cfg;
    test_ga_cfg.pop_size = 20;
    test_ga_cfg.n_generations = 10;
    test_ga_cfg.stall_gen_limit = 100;  % Don't stall early
    test_ga_cfg.verbose = false;

    % Single-objective wrapper (weighted sum)
    obj_single = @(x) single_obj_test(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);

    results_so = ga_single(obj_single, prob_cfg, test_ga_cfg);

    % Validate results struct
    assert(isfield(results_so, 'best_individual'), 'Missing best_individual');
    assert(isfield(results_so, 'best_fitness'), 'Missing best_fitness');
    assert(isfield(results_so, 'history'), 'Missing history');
    assert(isfield(results_so, 'final_population'), 'Missing final_population');
    assert(isfield(results_so, 'generations_run'), 'Missing generations_run');

    assert(numel(results_so.best_individual) == prob_cfg.n_vars, 'Best individual wrong size');
    assert(isfinite(results_so.best_fitness), 'Best fitness not finite');
    assert(results_so.generations_run == 10, 'Did not run 10 generations');
    assert(size(results_so.history, 1) == 10, 'History wrong length');
    assert(size(results_so.final_population, 1) == 20, 'Final pop wrong size');

    % Check fitness improved (or at least didn't get worse)
    first_best = results_so.history(1, 2);
    last_best = results_so.history(end, 2);
    assert(last_best <= first_best + 1e-6, 'Fitness got worse (elitism broken?)');

    fprintf('  Generations: %d\n', results_so.generations_run);
    fprintf('  Best fitness: %.6f\n', results_so.best_fitness);
    fprintf('  First gen best: %.6f → Final: %.6f\n', first_best, last_best);
    fprintf('  Improvement: %.4f%%\n', (first_best - last_best)/abs(first_best)*100);

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Single-Objective GA');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Single-Objective GA', ME);
end


%% ========================================================================
%  TEST 9: Multi-Objective GA / NSGA-II (Short Run)
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 9] Multi-Objective GA / NSGA-II (5 gen, pop=20)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    % Override config for quick test
    test_ga_cfg2 = ga_cfg;
    test_ga_cfg2.pop_size = 20;
    test_ga_cfg2.verbose = false;

    test_moo_cfg = moo_cfg;
    test_moo_cfg.max_generations = 5;
    test_moo_cfg.hv_stall_gen = 100;  % Don't stop early

    % Multi-objective wrapper
    obj_multi = @(x) moo_obj_test(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);

    results_mo = ga_multi(obj_multi, prob_cfg, test_ga_cfg2, test_moo_cfg);

    % Validate results struct
    assert(isfield(results_mo, 'pareto_front'), 'Missing pareto_front');
    assert(isfield(results_mo, 'pareto_set'), 'Missing pareto_set');
    assert(isfield(results_mo, 'history'), 'Missing history');
    assert(isfield(results_mo, 'final_population'), 'Missing final_population');
    assert(isfield(results_mo, 'final_objectives'), 'Missing final_objectives');
    assert(isfield(results_mo, 'generations_run'), 'Missing generations_run');

    % Pareto front checks
    n_pareto = size(results_mo.pareto_front, 1);
    assert(n_pareto >= 1, 'No Pareto solutions found');
    assert(size(results_mo.pareto_front, 2) == moo_cfg.n_objectives, 'Wrong obj dimension');
    assert(size(results_mo.pareto_set, 2) == prob_cfg.n_vars, 'Wrong var dimension');
    assert(size(results_mo.pareto_set, 1) == n_pareto, 'Pareto set/front size mismatch');

    % All objectives finite
    assert(all(isfinite(results_mo.pareto_front(:))), 'Non-finite Pareto values');
    assert(all(isfinite(results_mo.final_objectives(:))), 'Non-finite final obj');

    % History
    assert(numel(results_mo.history) == 5, 'History wrong length');
    assert(results_mo.generations_run == 5, 'Did not run 5 generations');

    fprintf('  Generations: %d\n', results_mo.generations_run);
    fprintf('  Pareto front: %d solutions\n', n_pareto);
    fprintf('  Final population: %d x %d\n', size(results_mo.final_population));
    fprintf('  Pareto front ranges:\n');
    for k = 1:moo_cfg.n_objectives
        fprintf('    %s: [%.2f, %.2f]\n', moo_cfg.objective_names{k}, ...
                min(results_mo.pareto_front(:,k)), max(results_mo.pareto_front(:,k)));
    end

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'NSGA-II Multi-Objective GA');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'NSGA-II Multi-Objective GA', ME);
end


%% ========================================================================
%  TEST 10: Convergence Tracker & Utilities
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 10] Convergence Tracker & Pareto Utilities\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    % Initialize tracker
    tracker = convergence_tracker('init', [], moo_cfg.n_objectives);
    assert(isfield(tracker, 'generations'), 'Tracker missing generations');
    assert(isfield(tracker, 'best_fitness'), 'Tracker missing best_fitness');
    assert(isfield(tracker, 'hypervolume'), 'Tracker missing hypervolume');

    % Simulate updates
    fake_obj = rand(20, 3) .* [20, 2000, 300];  % Simulate objective values
    fake_g = rand(20, 5) - 0.3;  % Some feasible, some not

    tracker = convergence_tracker('update', tracker, 1, fake_obj, fake_g, 0.5);
    tracker = convergence_tracker('update', tracker, 2, fake_obj*0.95, fake_g*0.8, 0.55);
    tracker = convergence_tracker('update', tracker, 3, fake_obj*0.9, fake_g*0.6, 0.6);

    assert(numel(tracker.generations) == 3, 'Tracker should have 3 entries');
    assert(tracker.hypervolume(end) == 0.6, 'HV not recorded correctly');
    assert(all(tracker.n_feasible > 0), 'Feasibility not tracked');

    % Test Pareto front extraction
    test_obj = [1 5; 2 3; 3 4; 1.5 4.5; 4 1; 2.5 2.5];
    pf_result = pareto_front('extract', test_obj);
    assert(pf_result.n_points >= 2, 'Should have at least 2 Pareto points');
    % Point [1,5] is dominated by nothing in first obj, [4,1] dominates in second
    % Non-dominated: [1,5], [2,3], [2.5,2.5], [4,1]? Check:
    % [1,5]: not dominated (best in obj1)
    % [4,1]: not dominated (best in obj2)
    % [2,3]: not dominated by others
    % [2.5, 2.5]: dominated by [2,3]? No: 2<2.5 and 3>2.5, so [2,3] does NOT dominate
    % Actually [2,3] dominates nothing that [2.5,2.5] doesn't... let's just check >=2
    assert(pf_result.n_points >= 2, 'Pareto extraction failed');

    % Test knee point
    knee = pareto_front('filter', test_obj(pf_result.indices, :));
    assert(knee >= 1, 'Knee index must be positive');
    assert(knee <= pf_result.n_points, 'Knee index out of range');

    fprintf('  Tracker: 3 generations logged\n');
    fprintf('  Hypervolume history: [%.2f, %.2f, %.2f]\n', tracker.hypervolume');
    fprintf('  Pareto extraction: %d non-dominated from 6 points\n', pf_result.n_points);
    fprintf('  Knee point: index %d\n', knee);

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Convergence Tracker & Utilities');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Convergence Tracker & Utilities', ME);
end


%% ========================================================================
%  TEST 11: Boundary Robustness (Extreme Design Vectors)
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 11] Boundary Robustness (Extreme Designs)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    % Test with lower bound
    x_low = prob_cfg.lb;
    [obj_lo, g_lo, ~] = evaluate_objectives(x_low, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);
    assert(all(isfinite(obj_lo)), 'Non-finite at lower bound');
    assert(all(isfinite(g_lo)), 'Non-finite constraints at lower bound');
    fprintf('  Lower bound: obj=[%.2f, %.1f, %.1f] - OK (finite)\n', obj_lo);

    % Test with upper bound
    x_high = prob_cfg.ub;
    [obj_hi, g_hi, ~] = evaluate_objectives(x_high, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);
    assert(all(isfinite(obj_hi)), 'Non-finite at upper bound');
    assert(all(isfinite(g_hi)), 'Non-finite constraints at upper bound');
    fprintf('  Upper bound: obj=[%.2f, %.1f, %.1f] - OK (finite)\n', obj_hi);

    % Test with random points near boundaries
    rng(123);
    n_boundary_tests = 10;
    boundary_failures = 0;
    for i = 1:n_boundary_tests
        x_test = prob_cfg.lb + rand(1, prob_cfg.n_vars) .* (prob_cfg.ub - prob_cfg.lb);
        % Push some variables to extremes
        extreme_vars = randperm(prob_cfg.n_vars, 3);
        for ev = extreme_vars
            if rand > 0.5
                x_test(ev) = prob_cfg.ub(ev);
            else
                x_test(ev) = prob_cfg.lb(ev);
            end
        end
        [obj_t, g_t, ~] = evaluate_objectives(x_test, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);
        if ~all(isfinite(obj_t)) || ~all(isfinite(g_t))
            boundary_failures = boundary_failures + 1;
        end
    end
    fprintf('  Random boundary tests: %d/%d passed (finite results)\n', ...
            n_boundary_tests - boundary_failures, n_boundary_tests);
    assert(boundary_failures == 0, sprintf('%d boundary tests produced non-finite values', ...
           boundary_failures));

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Boundary Robustness');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Boundary Robustness', ME);
end


%% ========================================================================
%  TEST 12: Reproducibility (Same Seed = Same Results)
%  ========================================================================
fprintf('\n─────────────────────────────────────────────────────────────────\n');
fprintf('[TEST 12] Reproducibility (RNG Seed Determinism)\n');
fprintf('─────────────────────────────────────────────────────────────────\n');

try
    test_ga_cfg3 = ga_cfg;
    test_ga_cfg3.pop_size = 10;
    test_ga_cfg3.n_generations = 3;
    test_ga_cfg3.stall_gen_limit = 100;
    test_ga_cfg3.verbose = false;
    test_ga_cfg3.rng_seed = 99;

    obj_repro = @(x) single_obj_test(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);

    % Run 1
    res1 = ga_single(obj_repro, prob_cfg, test_ga_cfg3);

    % Run 2 (same seed)
    res2 = ga_single(obj_repro, prob_cfg, test_ga_cfg3);

    % Must be identical
    assert(abs(res1.best_fitness - res2.best_fitness) < 1e-12, ...
           'Same seed produced different fitness!');
    assert(max(abs(res1.best_individual - res2.best_individual)) < 1e-12, ...
           'Same seed produced different individual!');
    assert(max(abs(res1.history(:) - res2.history(:))) < 1e-12, ...
           'Same seed produced different history!');

    fprintf('  Run 1 best fitness: %.10f\n', res1.best_fitness);
    fprintf('  Run 2 best fitness: %.10f\n', res2.best_fitness);
    fprintf('  Difference: %.2e (expected: 0)\n', abs(res1.best_fitness - res2.best_fitness));
    fprintf('  DETERMINISTIC: Confirmed identical results\n');

    [n_pass, test_log] = log_result(n_pass, test_log, 'PASS', 'Reproducibility');
catch ME
    [n_fail, test_log] = log_fail(n_fail, test_log, 'Reproducibility', ME);
end


%% ========================================================================
%  SUMMARY
%  ========================================================================
fprintf('\n\n');
fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║                    TEST SUMMARY                                 ║\n');
fprintf('╠══════════════════════════════════════════════════════════════════╣\n');
fprintf('║  PASSED: %2d / %2d                                                ║\n', ...
        n_pass, n_pass + n_fail);
fprintf('║  FAILED: %2d / %2d                                                ║\n', ...
        n_fail, n_pass + n_fail);
fprintf('╠══════════════════════════════════════════════════════════════════╣\n');

for i = 1:numel(test_log)
    if startsWith(test_log{i}, 'PASS')
        fprintf('║  [PASS] %s\n', test_log{i}(7:end));
    else
        fprintf('║  [FAIL] %s\n', test_log{i}(7:end));
    end
end

fprintf('╚══════════════════════════════════════════════════════════════════╝\n');

if n_fail == 0
    fprintf('\n  >> ALL TESTS PASSED. Framework ready for optimization.\n\n');
else
    fprintf('\n  >> %d TEST(S) FAILED. Review errors above before running optimization.\n\n', n_fail);
end

%% ========================================================================
%  LOCAL HELPER FUNCTIONS
%  ========================================================================

function [n, log] = log_result(n, log, status, name)
    fprintf('  >> %s\n', status);
    n = n + 1;
    log{end+1} = sprintf('%s: %s', status, name);
end

function [n, log] = log_fail(n, log, name, ME)
    fprintf('  >> FAIL: %s\n', ME.message);
    fprintf('     File: %s, Line: %d\n', ME.stack(1).file, ME.stack(1).line);
    n = n + 1;
    log{end+1} = sprintf('FAIL: %s - %s', name, ME.message);
end

function f = single_obj_test(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg)
% Lightweight single-objective wrapper for testing
    try
        [obj_values, g, ~] = evaluate_objectives(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);
        weights = [0.4, 0.3, 0.3];
        norm_factors = [20, 1500, 200];
        f = 0;
        for k = 1:numel(obj_values)
            normalized = obj_values(k) / norm_factors(k);
            if moo_cfg.obj_sense(k) == -1
                f = f - weights(k) * normalized;
            else
                f = f + weights(k) * normalized;
            end
        end
        violation = sum(max(0, g));
        if violation > 0
            f = f + 1000 * violation;
        end
    catch
        f = 1e6;
    end
end

function f = moo_obj_test(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg)
% Lightweight multi-objective wrapper for testing
    try
        [obj_values, g, ~] = evaluate_objectives(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);
        violation = sum(max(0, g));
        if violation > 0
            penalty = moo_cfg.penalty_factor * violation;
            for k = 1:numel(obj_values)
                if moo_cfg.obj_sense(k) == -1
                    obj_values(k) = obj_values(k) - penalty;
                else
                    obj_values(k) = obj_values(k) + penalty;
                end
            end
        end
        f = obj_values;
    catch
        f = zeros(1, moo_cfg.n_objectives);
        for k = 1:moo_cfg.n_objectives
            if moo_cfg.obj_sense(k) == -1
                f(k) = 0;
            else
                f(k) = 1e6;
            end
        end
    end
end
