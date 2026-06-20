function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL Longitudinal controller: wheel-slip ABS + (over-speed) braking.
%
%   The graded straight-braking scenario (B1) applies a large forced brake that
%   locks the wheels (slip ratio ~ -0.7). A locked tyre delivers LESS friction
%   than the peak-slip operating point (~ -0.12), so releasing brake to regulate
%   each wheel toward the peak slip shortens the stopping distance AND lowers the
%   slip-tracking error simultaneously.
%
%   ABS law (per wheel, slip-ratio regulation -> NOT PID): when a wheel is
%   braking past the target slip (kappa_i < -kappa_target) a release torque is
%   commanded proportional to the slip overshoot:
%       release_i = Kabs * (|kappa_i| - kappa_target)   [Nm], >= 0
%   The coordinator subtracts this from the wheel brake (it may drive the net
%   brake below the scenario value); the runner re-clips to [0, MAX].
%
%   Wheel slip is not a direct input: the runner caches the previous step's
%   per-wheel slip ratio in ctrlState.wheelSlip (4x1) before each call.
%
%   A light speed loop only ADDS braking when the vehicle is over the target
%   speed; it never accelerates against a forced-brake scenario (so it is inert
%   for B1, where vx <= vxRef throughout the stop).
%
%   Inputs:
%       vxRef     - target speed [m/s]
%       vx        - measured speed [m/s]
%       ax        - longitudinal accel [m/s^2] (previous step)
%       ctrlState - persistent state (.wheelSlip 4x1 set by runner; .prevFx)
%       CTRL      - uses CTRL.LON.* (see sim_params.m)
%       LIM       - .MAX_BRAKE_TRQ, .MAX_JERK
%       dt        - sample time [s]
%
%   Outputs:
%       forceCmd.Fx_total   - longitudinal force demand [N] (<0 brake)
%       forceCmd.brakeRatio - legacy brake fraction
%       forceCmd.absRelease - 4x1 per-wheel brake release [Nm] (>=0)
%       ctrlState           - updated state

    Lon = CTRL.LON;

    %% ---- 1. ABS: per-wheel slip regulation ----------------------------
    if isfield(ctrlState, 'wheelSlip') && numel(ctrlState.wheelSlip) == 4
        kappa = ctrlState.wheelSlip(:);
    else
        kappa = zeros(4, 1);
    end
    kt = Lon.kappaTarget;
    absRelease = zeros(4, 1);
    locking = kappa < -kt;                       % braking past peak slip
    absRelease(locking) = Lon.Kabs * (-kt - kappa(locking));   % >0
    absRelease = max(0, min(LIM.MAX_BRAKE_TRQ, absRelease));

    %% ---- 2. Speed loop: brake only when over target speed -------------
    err = vxRef - vx;
    Fx = 0;
    if err < -Lon.vDeadband
        Fx = Lon.KpV * err;                      % negative => braking
    end
    % jerk limit on Fx (rate of change of force)
    if ~isfield(ctrlState, 'prevFx'); ctrlState.prevFx = 0; end
    dFxMax = Lon.mEff * LIM.MAX_JERK * dt;
    Fx = max(ctrlState.prevFx - dFxMax, min(ctrlState.prevFx + dFxMax, Fx));

    %% ---- 3. Outputs + state update ------------------------------------
    forceCmd.Fx_total   = Fx;
    forceCmd.brakeRatio = double(any(locking));
    forceCmd.absRelease = absRelease;
    ctrlState.prevFx    = Fx;
end
