function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator allocation: map high-level commands to actuators.
%
%   Converts the lateral (AFS steer + ESC yaw moment), longitudinal (brake
%   force) and vertical (damping) commands into physical actuator demands:
%   road-wheel steer, 4-wheel brake torque [FL;FR;RL;RR] and 4-wheel damping.
%
%   Yaw-moment -> differential brake (sign convention: +Mz = CCW, body frame
%   x fwd / y left / z up). A braking force on the LEFT wheels produces +Mz,
%   on the RIGHT wheels -Mz. The requested Mz is split front/rear by ratioF and
%   realised as extra brake torque on the appropriate side:
%       T = |Mz_axle| * rw / (track/2)
%
%   NOTE: the returned brakeTorque is the CONTROLLER-added torque; the runner
%   adds it on top of the scenario (driver) brake before clipping.
%
%   Inputs:
%       latCmd.steerAngle - AFS additive steer [rad]
%       latCmd.yawMoment  - requested yaw moment [Nm]
%       lonCmd.Fx_total   - longitudinal force demand [N] (<0 braking)
%       lonCmd.brakeRatio - brake fraction (unused here; ABS handled upstream)
%       verCmd            - 4x1 damping [Ns/m]
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle   - final additive steer [rad], saturated
%       actuatorCmd.brakeTorque  - 4x1 [Nm], >=0, saturated to LIM.MAX_BRAKE_TRQ
%       actuatorCmd.dampingCoeff - 4x1 [Ns/m]

    rw   = VEH.rw;
    tf2  = VEH.track_f / 2;
    tr2  = VEH.track_r / 2;
    C    = CTRL.COORD;
    ratioF = local_get(C, 'ratioF', 0.5);   % front share of yaw moment

    brake = zeros(4, 1);   % [FL; FR; RL; RR]

    %% ---- 1. Longitudinal braking: distribute Fx_total, front/rear bias ----
    if isfield(lonCmd, 'Fx_total') && lonCmd.Fx_total < 0
        Fbrake = -lonCmd.Fx_total;          % total brake force [N] (>0)
        Tbrake = Fbrake * rw;               % total brake torque [Nm]
        bias_f = local_get(C, 'brakeBiasF', 0.6);
        brake(1) = brake(1) + Tbrake * bias_f / 2;       % FL
        brake(2) = brake(2) + Tbrake * bias_f / 2;       % FR
        brake(3) = brake(3) + Tbrake * (1 - bias_f) / 2; % RL
        brake(4) = brake(4) + Tbrake * (1 - bias_f) / 2; % RR
    end

    %% ---- 2. ESC yaw moment -> differential brake -------------------------
    Mz = 0;
    if isfield(latCmd, 'yawMoment'); Mz = latCmd.yawMoment; end
    if abs(Mz) > 1e-6
        Mz_f = ratioF * Mz;
        Mz_r = (1 - ratioF) * Mz;
        Tf = abs(Mz_f) * rw / tf2;
        Tr = abs(Mz_r) * rw / tr2;
        if Mz > 0
            brake(1) = brake(1) + Tf;   % FL  (+Mz -> brake left)
            brake(3) = brake(3) + Tr;   % RL
        else
            brake(2) = brake(2) + Tf;   % FR  (-Mz -> brake right)
            brake(4) = brake(4) + Tr;   % RR
        end
    end

    %% ---- 3. ABS release: subtract per-wheel release (may go negative) ----
    %   A negative net brake here lets the runner reduce the forced scenario
    %   brake (runner adds this to the scenario brake, then clips to [0, MAX]).
    if isfield(lonCmd, 'absRelease') && numel(lonCmd.absRelease) == 4
        brake = brake - lonCmd.absRelease(:);
    end

    %% ---- 4. Saturation (allow negative for ABS release) ------------------
    brake = max(-LIM.MAX_BRAKE_TRQ, min(LIM.MAX_BRAKE_TRQ, brake));

    %% ---- 5. Steer pass-through (safety saturation) -----------------------
    steer = latCmd.steerAngle;
    steer = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer));

    %% ---- 6. Assemble -----------------------------------------------------
    actuatorCmd.steerAngle   = steer;
    actuatorCmd.brakeTorque  = brake;
    actuatorCmd.dampingCoeff = verCmd;
end

function v = local_get(s, f, def)
    if isfield(s, f); v = s.(f); else; v = def; end
end
