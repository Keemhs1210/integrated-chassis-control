function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL Integrated lateral controller (AFS + ESC) via blended gain-scheduled LQI.
%
%   Two LQI designs on the MIMO 2-DOF bicycle model are blended by a smooth
%   sideslip gate sigma(|beta|):
%     - K_nom (nominal, steer-only, light beta penalty): gentle yaw-rate
%       tracking for the linear region -> does not fight a closed-loop driver
%       (DLC) and avoids over-damping the step response.
%     - K_lim (limit, two-input [delta; Mz], heavy beta penalty): aggressive
%       sideslip stabilisation for the handling limit (e.g. brake-in-turn).
%   Final law:
%       delta_AFS = (1-sigma)*(-K_nom z) + sigma*(-K_lim z)(1)
%       Mz        =  sigma * (-K_lim z)(2)
%   so the ESC yaw moment and the heavy beta correction only engage near the
%   limit, while normal driving stays gently steer-assisted.
%
%   State (augmented):  z = [beta; r; xi],  xi_dot = r - yawRateRef
%   The continuous Riccati equation for each gain is solved in-house via a
%   Hamiltonian eigen-decomposition (base MATLAB only, no toolbox). Gains are
%   recomputed as vx varies (gain scheduling / LPV) and cached.
%
%   Inputs:
%       yawRateRef - target yaw rate [rad/s]
%       yawRate    - measured yaw rate [rad/s]
%       slipAngle  - body sideslip beta [rad]
%       vx         - longitudinal speed [m/s]
%       ctrlState  - persistent state (.intError=xi; .Kn/.Kl/.vxc cache)
%       CTRL       - gains; uses CTRL.LAT.* (see sim_params.m)
%       LIM        - actuator limits
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS additive road-wheel steer [rad]
%       deltaAdd.yawMoment  - requested yaw moment [Nm] (coordinator -> brake diff)
%       ctrlState           - updated state

    L = CTRL.LAT;
    v = max(vx, L.vMin);

    %% ---- 1. Integral state (LQI): xi_dot = r - r_ref -------------------
    if ~isfield(ctrlState, 'intError'); ctrlState.intError = 0; end
    xi = ctrlState.intError + (yawRate - yawRateRef) * dt;
    xi = max(-L.intMax, min(L.intMax, xi));   % anti-windup clamp

    %% ---- 2. Gain-scheduled gains K_nom, K_lim (cached by vx) -----------
    needRecompute = ~isfield(ctrlState, 'Kn') || ~isfield(ctrlState, 'vxc') || ...
                    abs(v - ctrlState.vxc) > L.vSchedTol;
    if needRecompute
        [A, B] = local_bicycle_2in(v, L.veh);
        Aaug = [A, [0; 0]; 0 1 0];
        Baug = [B; 0 0];
        % nominal: steer-only (use first input column), light beta penalty
        Qn = diag([L.q_beta_nom, L.q_r_nom, L.q_xi_nom]);
        Kn = local_lqr_hamiltonian(Aaug, Baug(:, 1), Qn, L.r_delta_nom);
        % limit: two-input, heavy beta penalty
        Ql = diag([L.q_beta, L.q_r, L.q_xi]);
        Rl = diag([L.r_delta, L.r_Mz]);
        Kl = local_lqr_hamiltonian(Aaug, Baug, Ql, Rl);
        ctrlState.Kn = Kn; ctrlState.Kl = Kl; ctrlState.vxc = v;
    else
        Kn = ctrlState.Kn; Kl = ctrlState.Kl;
    end

    %% ---- 3. Blended control law ---------------------------------------
    z = [slipAngle; yawRate; xi];
    delta_nom = -Kn * z;            % scalar (steer only)
    u_lim     = -Kl * z;            % [delta; Mz]

    g  = (abs(slipAngle) - L.betaTh) / L.betaGate;   % beta gate
    g  = max(0, min(1, g));
    sigma = g * g * (3 - 2 * g);                     % smoothstep

    delta_AFS = (1 - sigma) * delta_nom + sigma * u_lim(1);
    Mz        = sigma * u_lim(2);

    %% ---- 4. Saturation -------------------------------------------------
    delta_AFS = max(-L.afsMax, min(L.afsMax, delta_AFS));
    Mz        = max(-L.MzMax,  min(L.MzMax,  Mz));

    %% ---- 5. Outputs + state update ------------------------------------
    deltaAdd.steerAngle = delta_AFS;
    deltaAdd.yawMoment  = Mz;
    ctrlState.intError  = xi;
    ctrlState.prevError = yawRateRef - yawRate;
end

%% ====================================================================
function [A, B] = local_bicycle_2in(vx, veh)
% 2-DOF bicycle, state [beta; r], input [delta; Mz].
    m = veh.m; Iz = veh.Iz; lf = veh.lf; lr = veh.lr; Cf = veh.Cf; Cr = veh.Cr;
    a11 = -(Cf + Cr) / (m * vx);
    a12 = -1 + (Cr * lr - Cf * lf) / (m * vx^2);
    a21 = (Cr * lr - Cf * lf) / Iz;
    a22 = -(Cf * lf^2 + Cr * lr^2) / (Iz * vx);
    A = [a11, a12; a21, a22];
    B = [Cf / (m * vx), 0; Cf * lf / Iz, 1 / Iz];
end

function K = local_lqr_hamiltonian(A, B, Q, R)
% Continuous-time LQR via Hamiltonian eigen-decomposition (base MATLAB only).
%   Solves A'P + PA - PBR^-1B'P + Q = 0 for stabilizing P, then K = R^-1 B' P.
    n = size(A, 1);
    H = [A, -B * (R \ B'); -Q, -A'];
    [V, D] = eig(H);
    d = diag(D);
    stable = real(d) < 0;
    Vs = V(:, stable);
    X1 = Vs(1:n, :);
    X2 = Vs(n+1:2*n, :);
    P = real(X2 / X1);
    P = (P + P') / 2;
    K = R \ (B' * P);
end
