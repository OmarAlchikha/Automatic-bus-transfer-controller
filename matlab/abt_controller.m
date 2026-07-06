function [k_main, k_apu, k_batt, connected, phase] = abt_controller(v_main, v_apu, v_batt)
%ABT_CONTROLLER  Automatic bus transfer logic (MATLAB Function block source).
%
%   Priority selection MAIN > APU > BATT with per-source undervoltage
%   detection (hysteresis + qualification time) and break-before-make
%   transfer with an enforced dead time.  This file is injected into the
%   "ABT Controller" MATLAB Function block by build_abt_model.m; it runs at
%   a fixed 1 kHz rate (inputs pass through 1 ms zero-order holds).
%
%   Mirrors python/abt_sim.py::Controller -- keep the two in sync.
%
%   Inputs are the SOURCE-SIDE terminal voltages, sensed between each
%   machine and its contactor, so an off-line source can be judged healthy
%   from its open-circuit voltage before being connected.
%
%   connected: -1 none, 0 MAIN, 1 APU, 2 BATT.   phase: 0 CONNECTED, 1 DEAD_TIME.
%#codegen

% Constants (mirror abt_params.m / python Params)
TS           = 1e-3;
V_FAIL       = [18 18 18];
V_OK         = [25 25 22];   % battery recovery threshold is lower: a 24 V
                             % battery never reaches a generator's 25 V band
T_FAIL_QUAL  = 0.050;
T_RECOV_QUAL = 1.000;
T_DEAD       = 0.050;
CONNECTED_PH = 0;  DEAD_TIME_PH = 1;

persistent healthy t_fail t_recov conn ph t_dead cmd
if isempty(healthy)
    healthy = [true true true];   % pre-start checks assumed passed
    t_fail  = [0 0 0];
    t_recov = [0 0 0];
    conn    = int8(-1);
    ph      = int8(DEAD_TIME_PH);
    t_dead  = T_DEAD;             % pre-elapsed: first source closes at t=0
    cmd     = [false false false];
end

v = [v_main, v_apu, v_batt];

% --- Per-source health: hysteresis + qualification timers ----------------
for i = 1:3
    if healthy(i)
        if v(i) < V_FAIL(i)
            t_fail(i) = t_fail(i) + TS;
            if t_fail(i) >= T_FAIL_QUAL
                healthy(i) = false;
                t_recov(i) = 0;
            end
        else
            t_fail(i) = 0;
        end
    else
        if v(i) > V_OK(i)
            t_recov(i) = t_recov(i) + TS;
            if t_recov(i) >= T_RECOV_QUAL
                healthy(i) = true;
                t_fail(i) = 0;
            end
        else
            t_recov(i) = 0;
        end
    end
end

% --- Highest-priority healthy source (fixed order MAIN > APU > BATT) -----
best = int8(-1);
for i = 1:3
    if healthy(i)
        best = int8(i - 1);
        break
    end
end

% --- Transfer sequencing: break first, wait dead time, then make ---------
if ph == CONNECTED_PH
    if best ~= conn
        % connected source failed, or a higher-priority source re-qualified
        cmd    = [false false false];
        conn   = int8(-1);
        ph     = int8(DEAD_TIME_PH);
        t_dead = 0;
    end
else
    cmd    = [false false false];
    t_dead = t_dead + TS;
    % target re-evaluated every sample: a source failing during the dead
    % time (cascading fault) is never connected
    if t_dead >= T_DEAD && best >= 0
        cmd            = [false false false];
        cmd(best + 1)  = true;
        conn           = best;
        ph             = int8(CONNECTED_PH);
    end
end

k_main    = cmd(1);
k_apu     = cmd(2);
k_batt    = cmd(3);
connected = double(conn);
phase     = double(ph);
end
