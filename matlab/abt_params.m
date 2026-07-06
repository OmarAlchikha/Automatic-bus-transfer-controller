function P = abt_params()
%ABT_PARAMS  Plant and controller parameters for the ABT model.
%   Single source of truth for the MATLAB side.  Values mirror
%   python/abt_sim.py (Params) -- keep the two in sync.  The controller
%   thresholds are duplicated as compile-time constants inside
%   abt_controller.m (a MATLAB Function block cannot cheaply read a
%   workspace struct); that file states the same values.

% --- Sources: [MAIN APU BATT] -------------------------------------------
P.emf_set = [28.5 28.2 25.6];   % EMF setpoints [V]
P.r_src   = [0.025 0.035 0.040];% source series resistance [ohm]
P.tau_emf = 0.050;              % generator field/regulator lag [s]

% --- Bus & loads ----------------------------------------------------------
P.c_bus  = 0.020;               % bus hold-up capacitance [F]
P.r_load = 1.4;                 % base load (~20 A at 28 V) [ohm]
P.r_step = 2.0;                 % step load (~+14 A) [ohm]

% --- Contactors -----------------------------------------------------------
P.t_contactor = 0.020;          % actuation delay [s]

% --- Controller (also hard-coded in abt_controller.m) ---------------------
P.ts          = 1e-3;           % controller sample time [s]
P.v_fail      = [18 18 18];     % undervoltage threshold per source [V]
P.v_ok        = [25 25 22];     % recovery threshold per source [V]
P.t_fail_qual = 0.050;          % failure qualification time [s]
P.t_recov_qual= 1.000;          % recovery qualification time [s]
P.t_dead      = 0.050;          % break-before-make dead time [s]
end
