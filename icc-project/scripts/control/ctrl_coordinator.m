function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator allocation via weighted least squares + friction circle.
%
%   The high-level demands — desired longitudinal force (braking) and ESC yaw
%   moment — form a virtual control v = [Fx; Mz]. These are mapped to the four
%   wheel brake torques u = [T_FL; T_FR; T_RL; T_RR] by the effectiveness matrix
%       Fx = -(1/rw) * sum(T_i)
%       Mz =  (1/rw) * [ t_f/2, -t_f/2, t_r/2, -t_r/2 ] * u     (+Mz = CCW)
%   and solved by a damped weighted least squares (toolbox-free):
%       u* = (B' Wv B + Wu)^{-1} B' Wv v
%   Each wheel torque is then clipped to a FRICTION-CIRCLE budget so that the
%   combined longitudinal+lateral tyre force stays within mu*Fz:
%       T_max,i = rw * sqrt( max(0, (mu Fz_i)^2 - Fy_i^2) )
%   Fz_i is the static load plus a longitudinal load-transfer estimate; the
%   lateral force usage Fy_i is estimated from the ESC demand (the vehicle is
%   near the lateral limit when a large Mz is requested). This caps the ESC
%   differential brake so it never pushes a laterally-saturated tyre past grip.
%
%   ABS brake-release (lonCmd.absRelease) is then subtracted (may drive the net
%   brake negative so the runner can reduce the forced scenario brake).
%
%   Inputs:  latCmd.steerAngle/.yawMoment, lonCmd.Fx_total/.absRelease(4),
%            verCmd(4), vx, VEH, CTRL, LIM
%   Output:  actuatorCmd.steerAngle, .brakeTorque(4), .dampingCoeff(4)

    rw  = VEH.rw;
    tf2 = VEH.track_f / 2;
    tr2 = VEH.track_r / 2;
    C   = CTRL.COORD;
    mu  = local_get(C, 'mu', 1.0);

    %% ---- 1. Virtual control demand v = [Fx; Mz] -----------------------
    Fx_req = 0;
    if isfield(lonCmd, 'Fx_total') && lonCmd.Fx_total < 0
        Fx_req = lonCmd.Fx_total;            % [N] (<0 braking)
    end
    Mz_req = 0;
    if isfield(latCmd, 'yawMoment'); Mz_req = latCmd.yawMoment; end
    v = [Fx_req; Mz_req];

    %% ---- 2. Friction-circle budget per wheel --------------------------
    g = 9.81; m = VEH.mass; L = VEH.lf + VEH.lr; h = VEH.h_cog;
    % static vertical load per wheel
    Fz = [m*g*VEH.lr/L/2; m*g*VEH.lr/L/2; m*g*VEH.lf/L/2; m*g*VEH.lf/L/2];
    % longitudinal load transfer from the commanded decel (braking -> front +)
    ax_est = Fx_req / m;                      % [m/s^2] (<0)
    dFz = -m * ax_est * h / L;                % total transferred to front (>0)
    Fz = Fz + [ dFz/2; dFz/2; -dFz/2; -dFz/2 ];
    Fz = max(Fz, 50);                         % keep positive
    % lateral usage estimate: rises with ESC demand (near-limit cornering)
    MzRef = local_get(C, 'MzRef', 4000);
    latRes = local_get(C, 'latReserve', 0.7);
    k_lat = min(1, latRes * abs(Mz_req) / MzRef);   % fraction of grip used laterally
    Tmax = rw * mu * Fz * sqrt(max(0, 1 - k_lat^2));
    Tmax = min(Tmax, LIM.MAX_BRAKE_TRQ);

    %% ---- 3. WLS allocation v -> u (damped weighted least squares) -----
    B = [ -1/rw, -1/rw, -1/rw, -1/rw; ...
          tf2/rw, -tf2/rw, tr2/rw, -tr2/rw ];
    Wv = diag([local_get(C,'wFx',1), local_get(C,'wMz',1)]);
    Wu = local_get(C, 'wU', 1e-2) * eye(4);
    u = (B' * Wv * B + Wu) \ (B' * Wv * v);   % unconstrained optimum
    brake = max(0, min(Tmax, u));             % friction-circle + non-negativity

    %% ---- 4. ABS release: subtract per-wheel release (may go negative) -
    if isfield(lonCmd, 'absRelease') && numel(lonCmd.absRelease) == 4
        brake = brake - lonCmd.absRelease(:);
    end

    %% ---- 5. Saturation (allow negative for ABS release) --------------
    brake = max(-LIM.MAX_BRAKE_TRQ, min(LIM.MAX_BRAKE_TRQ, brake));

    %% ---- 6. Steer pass-through + assemble ----------------------------
    steer = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));
    actuatorCmd.steerAngle   = steer;
    actuatorCmd.brakeTorque  = brake;
    actuatorCmd.dampingCoeff = verCmd;
end

function v = local_get(s, f, def)
    if isfield(s, f); v = s.(f); else; v = def; end
end
