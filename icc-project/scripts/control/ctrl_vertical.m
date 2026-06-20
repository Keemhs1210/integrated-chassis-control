function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC (Continuous Damping Control) — per-wheel semi-active damping.
%
%   Semi-active dampers can only dissipate (force = c*(stroke rate), c in
%   [cMin,cMax]). True skyhook needs the ABSOLUTE sprung velocity; here only the
%   roll/pitch modal sprung velocity is measured (body heave is not observable
%   from the provided state). The law therefore combines:
%     (1) Skyhook on the observable modal sprung velocity (roll/pitch corners):
%         c_sky = skyGain * v_s_modal / v_rel   when v_s_modal*v_rel > 0.
%     (2) Relative-velocity adaptive damping for the (unobservable) heave mode:
%         c_adapt = Kv * |v_rel|   -> firms up at the body-bounce resonance where
%         stroke rate is large, softening elsewhere for comfort.
%   c = clip(cMin + max(c_sky_term, c_adapt), cMin, cMax).
%
%   Inputs:
%       suspState - .zs_dot(4) modal sprung corner vel (roll/pitch),
%                   .zu_dot(4) suspension stroke rate (relative), .zs/.zu (4)
%       ctrlState - persistent state (reserved)
%       CTRL      - CTRL.VER.cMin,.cMax,.skyGain,.Kv,.alpha
%       dt        - sample time [s]
%
%   Output:
%       dampingCmd - 4x1 damping coefficient [Ns/m], cMin <= c <= cMax

    V = CTRL.VER;
    cMin = V.cMin; cMax = V.cMax;
    skyGain = V.skyGain;
    Kv      = local_get(V, 'Kv', 12000);     % relative-velocity adaptive gain

    if ~isfield(suspState, 'zu_dot') || numel(suspState.zu_dot) ~= 4
        dampingCmd = 0.5 * (cMin + cMax) * ones(4, 1);
        return;
    end
    vrel = suspState.zu_dot(:);                       % stroke rate (damper acts on this)
    if isfield(suspState, 'zs_dot') && numel(suspState.zs_dot) == 4
        vsm = suspState.zs_dot(:);                    % observable modal sprung vel
    else
        vsm = zeros(4, 1);
    end

    dampingCmd = zeros(4, 1);
    for i = 1:4
        % (2) relative-velocity adaptive (resonance suppression, heave-capable)
        c_adapt = Kv * abs(vrel(i));
        % (1) skyhook on observable modal sprung velocity
        if abs(vrel(i)) > 1e-4 && vsm(i) * vrel(i) > 0
            c_sky = skyGain * vsm(i) / vrel(i);
        else
            c_sky = 0;
        end
        c = cMin + max(c_sky, c_adapt);
        dampingCmd(i) = max(cMin, min(cMax, c));
    end
end

function v = local_get(s, f, def)
    if isfield(s, f); v = s.(f); else; v = def; end
end
