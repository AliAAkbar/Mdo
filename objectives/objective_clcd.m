function f = objective_clcd(aero_results)
% OBJECTIVE_CLCD  Aerodynamic efficiency objective (CL/CD).
%   f = objective_clcd(aero_results)
%
%   Returns the lift-to-drag ratio at cruise condition.
%   The optimizer maximizes this (via obj_sense = -1).
%
%   Input:
%     aero_results - Struct from compute_aero()
%
%   Output:
%     f - Scalar CL/CD value (higher is better)
%         Returns 0 if aero results are invalid.

    if ~aero_results.valid
        f = 0;  % Penalize invalid configurations
        return;
    end

    % Direct L/D at cruise
    f = aero_results.CL_CD;

    % Sanity bounds (prevent unrealistic values)
    f = max(0, min(f, 30));  % Cap at 30 (unrealistic above this for amphibious)

end
