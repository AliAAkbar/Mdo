function vsp_output = run_openvsp(design_vars, prob_cfg, vsp_settings)
% RUN_OPENVSP  Generate geometry and run aerodynamic analysis via OpenVSP.
%   vsp_output = run_openvsp(design_vars, prob_cfg, vsp_settings)
%
%   This function:
%     1. Writes a .vspscript to update geometry parameters
%     2. Executes OpenVSP in batch mode (headless)
%     3. Runs VLM/Panel analysis (VSPAero)
%     4. Returns raw output file paths for parsing
%
%   Inputs:
%     design_vars  - [1 x n_vars] Design variable vector
%     prob_cfg     - Problem config struct (from problem_config.m)
%     vsp_settings - Struct with OpenVSP paths and analysis settings
%                    (from vsp_settings() or passed externally)
%
%   Output:
%     vsp_output   - Struct with fields:
%                    .success      - Logical (true if analysis completed)
%                    .polar_file   - Path to polar results file
%                    .history_file - Path to convergence history file
%                    .geom_file    - Path to exported geometry (.vsp3)
%                    .log          - Console output / error messages
%
%   NOTE: This function requires OpenVSP and vspaero to be installed and
%         accessible. Set paths in vsp_settings.

    % --- Default VSP settings if not provided ---
    if nargin < 3 || isempty(vsp_settings)
        vsp_settings = default_vsp_settings();
    end

    vsp_output = struct();
    vsp_output.success = false;
    vsp_output.polar_file = '';
    vsp_output.history_file = '';
    vsp_output.geom_file = '';
    vsp_output.log = '';

    % --- Map design variables to named parameters ---
    var_names = prob_cfg.var_names;
    params = struct();
    for i = 1:numel(var_names)
        params.(var_names{i}) = design_vars(i);
    end

    % --- Create working directory for this run ---
    run_id = sprintf('run_%s', datestr(now, 'yyyymmdd_HHMMSS_FFF'));
    work_dir = fullfile(vsp_settings.work_dir, run_id);
    if ~exist(work_dir, 'dir')
        mkdir(work_dir);
    end

    % --- Generate VSP script to update geometry ---
    script_file = fullfile(work_dir, 'update_geom.vspscript');
    generate_vsp_script(script_file, params, vsp_settings);

    % --- Run OpenVSP in batch mode ---
    vsp3_file = fullfile(work_dir, 'aircraft.vsp3');

    % Step 1: Update geometry using vspscript
    cmd_geom = sprintf('"%s" -script "%s" -des "%s" -o "%s"', ...
        vsp_settings.vsp_exec, script_file, vsp_settings.base_model, vsp3_file);

    [status1, output1] = system(cmd_geom);
    if status1 ~= 0
        vsp_output.log = sprintf('OpenVSP geometry update failed:\n%s', output1);
        warning('run_openvsp: Geometry generation failed. Run ID: %s', run_id);
        return;
    end

    % Step 2: Export DegenGeom for VSPAero
    degen_file = fullfile(work_dir, 'aircraft_DegenGeom.csv');
    cmd_degen = sprintf('"%s" -batch "%s" -degengeom', ...
        vsp_settings.vsp_exec, vsp3_file);

    [status2, output2] = system(cmd_degen);
    if status2 ~= 0
        vsp_output.log = sprintf('DegenGeom export failed:\n%s', output2);
        warning('run_openvsp: DegenGeom export failed. Run ID: %s', run_id);
        return;
    end

    % Step 3: Run VSPAero (VLM or Panel method)
    vspaero_input = write_vspaero_input(work_dir, params, vsp_settings);
    cmd_aero = sprintf('"%s" "%s"', vsp_settings.vspaero_exec, vspaero_input);

    [status3, output3] = system(cmd_aero);
    if status3 ~= 0
        vsp_output.log = sprintf('VSPAero analysis failed:\n%s', output3);
        warning('run_openvsp: VSPAero failed. Run ID: %s', run_id);
        return;
    end

    % --- Collect output files ---
    polar_file = fullfile(work_dir, 'aircraft.polar');
    history_file = fullfile(work_dir, 'aircraft.history');

    if exist(polar_file, 'file') && exist(history_file, 'file')
        vsp_output.success = true;
        vsp_output.polar_file = polar_file;
        vsp_output.history_file = history_file;
        vsp_output.geom_file = vsp3_file;
        vsp_output.log = output3;
    else
        vsp_output.log = 'Output files not found after VSPAero run.';
        warning('run_openvsp: Output files missing. Run ID: %s', run_id);
    end

end

%% ========================================================================
%  LOCAL HELPER FUNCTIONS
%  ========================================================================

function settings = default_vsp_settings()
% DEFAULT_VSP_SETTINGS  Default OpenVSP paths and analysis parameters.
    settings = struct();
    settings.vsp_exec       = 'vsp';           % OpenVSP executable
    settings.vspaero_exec   = 'vspaero';       % VSPAero solver executable
    settings.base_model     = 'base_aircraft.vsp3';  % Template model
    settings.work_dir       = fullfile(pwd, 'output', 'vsp_runs');
    settings.analysis_type  = 'VLM';           % 'VLM' or 'Panel'

    % Flight conditions
    settings.alpha_range    = [-2, 0, 2, 4, 6, 8, 10, 12]; % [deg] AoA sweep
    settings.mach           = 0.16;            % Mach number at cruise
    settings.reynolds       = 3e6;             % Reynolds number
    settings.sref           = 0;               % [m^2] 0 = auto from model
    settings.cref           = 0;               % [m] 0 = auto from model
    settings.bref           = 0;               % [m] 0 = auto from model

    % Solver settings
    settings.n_wake_iter    = 5;               % Wake iteration count
    settings.far_field_dist = 10;              % Far-field distance (in spans)
end

function generate_vsp_script(script_file, params, settings)
% GENERATE_VSP_SCRIPT  Write a VSPscript file to update geometry parameters.
    fid = fopen(script_file, 'w');
    if fid == -1
        error('Cannot open script file: %s', script_file);
    end

    fprintf(fid, '// Auto-generated VSP script for MDO\n');
    fprintf(fid, '// Generated: %s\n\n', datestr(now));

    % Open base model
    fprintf(fid, 'void main()\n{\n');
    fprintf(fid, '    ReadVSPFile("%s");\n\n', settings.base_model);

    % Update wing parameters
    fprintf(fid, '    // --- Wing geometry updates ---\n');
    fprintf(fid, '    string wing_id = FindGeomsWithName("Wing")[0];\n');
    fprintf(fid, '    SetParmVal(wing_id, "TotalSpan", "WingGeom", %.6f);\n', params.wing_span);
    fprintf(fid, '    SetParmVal(wing_id, "Root_Chord", "XSec_1", %.6f);\n', params.wing_chord_root);
    fprintf(fid, '    SetParmVal(wing_id, "Tip_Chord", "XSec_2", %.6f);\n', params.wing_chord_tip);
    fprintf(fid, '    SetParmVal(wing_id, "Sweep", "XSec_1", %.6f);\n', params.wing_sweep);
    fprintf(fid, '    SetParmVal(wing_id, "Twist", "XSec_2", %.6f);\n', params.wing_twist);
    fprintf(fid, '    SetParmVal(wing_id, "Dihedral", "XSec_1", %.6f);\n', params.wing_dihedral);
    fprintf(fid, '    SetParmVal(wing_id, "ThickChord", "XSecCurve_1", %.6f);\n', params.wing_tc_ratio);

    % Update hull/fuselage parameters
    fprintf(fid, '\n    // --- Hull geometry updates ---\n');
    fprintf(fid, '    string hull_id = FindGeomsWithName("Hull")[0];\n');
    fprintf(fid, '    SetParmVal(hull_id, "Width", "XSec_3", %.6f);\n', params.hull_beam);

    % Write and export
    fprintf(fid, '\n    // --- Update and export ---\n');
    fprintf(fid, '    Update();\n');
    fprintf(fid, '    ExportFile("aircraft.vsp3", SET_ALL, EXPORT_VSP);\n');
    fprintf(fid, '    ComputeDegenGeom(SET_ALL, DEGEN_GEOM_CSV_TYPE);\n');
    fprintf(fid, '}\n');

    fclose(fid);
end

function input_file = write_vspaero_input(work_dir, params, settings)
% WRITE_VSPAERO_INPUT  Write VSPAero .vspaero input file.
    input_file = fullfile(work_dir, 'aircraft');  % VSPAero uses filename without extension

    fid = fopen([input_file, '.vspaero'], 'w');
    if fid == -1
        error('Cannot write VSPAero input file.');
    end

    % Compute reference values from design variables
    S_ref = params.wing_span * (params.wing_chord_root + params.wing_chord_tip) / 2;
    c_ref = (params.wing_chord_root + params.wing_chord_tip) / 2;
    b_ref = params.wing_span;

    if settings.sref > 0; S_ref = settings.sref; end
    if settings.cref > 0; c_ref = settings.cref; end
    if settings.bref > 0; b_ref = settings.bref; end

    fprintf(fid, 'Sref = %.4f\n', S_ref);
    fprintf(fid, 'Cref = %.4f\n', c_ref);
    fprintf(fid, 'Bref = %.4f\n', b_ref);
    fprintf(fid, 'X_cg = %.4f\n', 0.25 * c_ref);  % Quarter chord CG
    fprintf(fid, 'Y_cg = 0.0\n');
    fprintf(fid, 'Z_cg = 0.0\n');
    fprintf(fid, 'Mach = %.4f\n', settings.mach);
    fprintf(fid, 'AoA = %.2f\n', settings.alpha_range(1));  % Starting AoA
    fprintf(fid, 'Beta = 0.0\n');
    fprintf(fid, 'Vinf = 1.0\n');
    fprintf(fid, 'Rho = 1.225\n');
    fprintf(fid, 'ReCref = %.0f\n', settings.reynolds);
    fprintf(fid, 'ClMax = -1.0\n');  % No Cl limiting
    fprintf(fid, 'MaxTurningAngle = -1.0\n');
    fprintf(fid, 'Symmetry = Y\n');

    % Analysis type
    if strcmpi(settings.analysis_type, 'Panel')
        fprintf(fid, 'AnalysisType = Panel\n');
    else
        fprintf(fid, 'AnalysisType = VLM\n');
    end

    fprintf(fid, 'WakeIters = %d\n', settings.n_wake_iter);
    fprintf(fid, 'FarDist = %.1f\n', settings.far_field_dist);

    % Alpha sweep
    fprintf(fid, 'NumberOfAlphas = %d\n', numel(settings.alpha_range));
    fprintf(fid, 'AlphaStart = %.2f\n', settings.alpha_range(1));
    fprintf(fid, 'AlphaEnd = %.2f\n', settings.alpha_range(end));

    fclose(fid);
end
