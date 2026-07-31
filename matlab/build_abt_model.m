function build_abt_model()
%BUILD_ABT_MODEL  Programmatically construct the ABT Simulink/Simscape model.
%
%   Creates and saves 'abt_model.slx' next to this file.  The model is
%   built entirely from Simscape Foundation electrical blocks plus a
%   MATLAB Function controller, so the whole design is reviewable as text
%   in this repo (no hand-edited binary is committed).
%
%   Topology (one branch per source, MAIN / APU / BATT):
%
%     [emf_* From Workspace] -> [field lag 1/(tau.s+1)] -> S-PS -> CVS
%     CVS(+) -- R_src -- Current Sensor -- (terminal node) -- Switch -- BUS
%                                             |
%                                     Voltage Sensor (source-side sensing,
%                                     upstream of the contactor)
%
%     BUS -- C_bus, R_load, (Switch + R_step) -- ground;  Voltage Sensor
%
%   Controller: MATLAB Function block (script injected from
%   abt_controller.m), fed through 1 ms zero-order holds; its contactor
%   commands pass through 20 ms transport delays that model contactor
%   actuation time.
%
%   Scenario inputs expected in the base workspace (see run_scenarios.m):
%     emf_main, emf_apu, emf_batt : timeseries of EMF targets [V]
%     load_step                   : timeseries 0/1 for the step load
%
%   NOTE on physical ports: connections use LConn/RConn names following
%   the documented port layout of the Foundation Library (verified against
%   R2023b docs).  If a future release renumbers a port, add_line raises
%   an error naming the offending connection -- fix the index there.

P   = abt_params();
mdl = 'abt_model';

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

set_param(mdl, 'Solver', 'ode23t', 'StopTime', '5', ...
    'MaxStep', '1e-3', 'RelTol', '1e-4', ...
    'SimscapeLogType', 'none');

FL = 'fl_lib/Electrical';                 % Simscape Foundation, electrical
NE = 'nesl_utility';                      % Simscape utilities

src   = {'Main', 'Apu', 'Batt'};
wsvar = {'emf_main', 'emf_apu', 'emf_batt'};
ybase = [100, 420, 740];                  % one row per source

pos = @(x, y, w, h) [x, y, x + w, y + h];

% ---------------------------------------------------------------- sources
for i = 1:3
    s = src{i};  y = ybase(i);

    add_block('simulink/Sources/From Workspace', [mdl '/EMF_' s], ...
        'VariableName', wsvar{i}, 'Position', pos(30, y, 60, 28));
    add_block('simulink/Continuous/Transfer Fcn', [mdl '/FieldLag_' s], ...
        'Numerator', '[1]', 'Denominator', sprintf('[%g 1]', P.tau_emf), ...
        'Position', pos(130, y, 70, 30));
    add_block([NE '/Simulink-PS Converter'], [mdl '/SPS_EMF_' s], ...
        'Position', pos(240, y, 40, 28));
    add_block([FL '/Electrical Sources/Controlled Voltage Source'], ...
        [mdl '/CVS_' s], 'Position', pos(320, y, 40, 40));
    add_block([FL '/Electrical Elements/Resistor'], [mdl '/Rsrc_' s], ...
        'R', sprintf('%g', P.r_src(i)), 'Position', pos(400, y, 40, 30));
    add_block([FL '/Electrical Sensors/Current Sensor'], [mdl '/Isen_' s], ...
        'Position', pos(480, y, 40, 30));
    add_block([FL '/Electrical Sensors/Voltage Sensor'], [mdl '/Vsen_' s], ...
        'Position', pos(540, y + 90, 40, 30));
    add_block([FL '/Electrical Elements/Switch'], [mdl '/K_' s], ...
        'Threshold', '0.5', 'Position', pos(640, y, 40, 30));
    add_block([FL '/Electrical Elements/Electrical Reference'], ...
        [mdl '/Gnd_' s], 'Position', pos(320, y + 90, 30, 30));
    add_block([FL '/Electrical Elements/Electrical Reference'], ...
        [mdl '/GndV_' s], 'Position', pos(600, y + 160, 30, 30));

    % EMF command chain (Simulink -> physical)
    add_line(mdl, ['EMF_' s '/1'], ['FieldLag_' s '/1']);
    add_line(mdl, ['FieldLag_' s '/1'], ['SPS_EMF_' s '/1']);
    add_line(mdl, ['SPS_EMF_' s '/RConn1'], ['CVS_' s '/LConn1']);

    % Electrical branch: CVS(+) -> Rsrc -> Isen -> terminal -> switch
    add_line(mdl, ['CVS_' s '/RConn1'], ['Rsrc_' s '/LConn1']);
    add_line(mdl, ['CVS_' s '/RConn2'], ['Gnd_' s '/LConn1']);
    add_line(mdl, ['Rsrc_' s '/RConn1'], ['Isen_' s '/LConn1']);
    add_line(mdl, ['Isen_' s '/RConn1'], ['K_' s '/LConn2']);

    % Source-side voltage sensing at the terminal node (upstream of switch)
    add_line(mdl, ['Isen_' s '/RConn1'], ['Vsen_' s '/LConn1']);
    add_line(mdl, ['Vsen_' s '/RConn1'], ['GndV_' s '/LConn1']);

    % Sensor physical outputs -> Simulink
    add_block([NE '/PS-Simulink Converter'], [mdl '/PSS_I_' s], ...
        'Position', pos(480, y + 90, 40, 28));
    add_block([NE '/PS-Simulink Converter'], [mdl '/PSS_V_' s], ...
        'Position', pos(620, y + 90, 40, 28));
    add_line(mdl, ['Isen_' s '/RConn2'], ['PSS_I_' s '/LConn1']);
    add_line(mdl, ['Vsen_' s '/RConn2'], ['PSS_V_' s '/LConn1']);

    add_block('simulink/Sinks/To Workspace', [mdl '/log_i_' lower(s)], ...
        'VariableName', ['i_' lower(s)], 'SaveFormat', 'Timeseries', ...
        'Position', pos(560, y + 130, 70, 26));
    add_block('simulink/Sinks/To Workspace', [mdl '/log_v_' lower(s)], ...
        'VariableName', ['v_' lower(s)], 'SaveFormat', 'Timeseries', ...
        'Position', pos(700, y + 130, 70, 26));
    add_line(mdl, ['PSS_I_' s '/1'], ['log_i_' lower(s) '/1']);
    add_line(mdl, ['PSS_V_' s '/1'], ['log_v_' lower(s) '/1']);
end

% -------------------------------------------------------------------- bus
yb = 420;
add_block([FL '/Electrical Elements/Capacitor'], [mdl '/C_bus'], ...
    'c', sprintf('%g', P.c_bus), 'Position', pos(900, yb, 40, 30));
add_block([FL '/Electrical Elements/Resistor'], [mdl '/R_load'], ...
    'R', sprintf('%g', P.r_load), 'Position', pos(960, yb, 40, 30));
add_block([FL '/Electrical Elements/Switch'], [mdl '/K_StepLoad'], ...
    'Threshold', '0.5', 'Position', pos(1020, yb, 40, 30));
add_block([FL '/Electrical Elements/Resistor'], [mdl '/R_step'], ...
    'R', sprintf('%g', P.r_step), 'Position', pos(1020, yb + 60, 40, 30));
add_block([FL '/Electrical Sensors/Voltage Sensor'], [mdl '/Vsen_Bus'], ...
    'Position', pos(1090, yb, 40, 30));
add_block([FL '/Electrical Elements/Electrical Reference'], ...
    [mdl '/Gnd_Bus'], 'Position', pos(960, yb + 150, 30, 30));
add_block([NE '/Solver Configuration'], [mdl '/SolverConfig'], ...
    'Position', pos(900, yb - 80, 40, 30));

% Contactor outputs all land on the bus node
for i = 1:3
    add_line(mdl, ['K_' src{i} '/RConn1'], 'C_bus/LConn1');
end
add_line(mdl, 'C_bus/LConn1', 'R_load/LConn1');
add_line(mdl, 'C_bus/LConn1', 'K_StepLoad/LConn2');
add_line(mdl, 'C_bus/LConn1', 'Vsen_Bus/LConn1');
add_line(mdl, 'C_bus/LConn1', 'SolverConfig/RConn1');
add_line(mdl, 'K_StepLoad/RConn1', 'R_step/LConn1');
add_line(mdl, 'C_bus/RConn1', 'Gnd_Bus/LConn1');
add_line(mdl, 'R_load/RConn1', 'Gnd_Bus/LConn1');
add_line(mdl, 'R_step/RConn1', 'Gnd_Bus/LConn1');
add_line(mdl, 'Vsen_Bus/RConn1', 'Gnd_Bus/LConn1');

add_block([NE '/PS-Simulink Converter'], [mdl '/PSS_V_Bus'], ...
    'Position', pos(1150, yb, 40, 28));
add_line(mdl, 'Vsen_Bus/RConn2', 'PSS_V_Bus/LConn1');
add_block('simulink/Sinks/To Workspace', [mdl '/log_v_bus'], ...
    'VariableName', 'v_bus', 'SaveFormat', 'Timeseries', ...
    'Position', pos(1220, yb, 70, 26));
add_line(mdl, 'PSS_V_Bus/1', 'log_v_bus/1');

% Step-load command
add_block('simulink/Sources/From Workspace', [mdl '/LoadStepCmd'], ...
    'VariableName', 'load_step', 'Position', pos(900, yb + 70, 60, 28));
add_block([NE '/Simulink-PS Converter'], [mdl '/SPS_LoadStep'], ...
    'Position', pos(975, yb + 70, 40, 28));
add_line(mdl, 'LoadStepCmd/1', 'SPS_LoadStep/1');
add_line(mdl, 'SPS_LoadStep/RConn1', 'K_StepLoad/LConn1');

% -------------------------------------------------------------- controller
yc = 1100;
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/ABT Controller'], 'Position', pos(400, yc, 180, 120));

% Inject the controller source (kept as a reviewable .m file in the repo)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/ABT Controller']);
ch.Script = fileread(fullfile(fileparts(mfilename('fullpath')), ...
    'abt_controller.m'));

% Inputs: source terminal voltages through 1 ms ZOHs (sets the 1 kHz rate
% the controller's internal timers assume)
for i = 1:3
    s = src{i};
    add_block('simulink/Discrete/Zero-Order Hold', [mdl '/ZOH_' s], ...
        'SampleTime', sprintf('%g', P.ts), ...
        'Position', pos(280, yc + 10 + 40 * (i - 1), 50, 26));
    add_line(mdl, ['PSS_V_' s '/1'], ['ZOH_' s '/1']);
    add_line(mdl, ['ZOH_' s '/1'], sprintf('ABT Controller/%d', i));
end

% Contactor commands through 20 ms actuation delays, then into the switches
for i = 1:3
    s = src{i};
    add_block('simulink/Continuous/Transport Delay', [mdl '/CtorDelay_' s], ...
        'DelayTime', sprintf('%g', P.t_contactor), ...
        'Position', pos(640, yc + 10 + 40 * (i - 1), 50, 26));
    add_block([NE '/Simulink-PS Converter'], [mdl '/SPS_K_' s], ...
        'Position', pos(720, yc + 10 + 40 * (i - 1), 40, 26));
    add_line(mdl, sprintf('ABT Controller/%d', i), ['CtorDelay_' s '/1']);
    add_line(mdl, ['CtorDelay_' s '/1'], ['SPS_K_' s '/1']);
    add_line(mdl, ['SPS_K_' s '/RConn1'], ['K_' s '/LConn1']);

    add_block('simulink/Sinks/To Workspace', [mdl '/log_k_' lower(s)], ...
        'VariableName', ['k_' lower(s)], 'SaveFormat', 'Timeseries', ...
        'Position', pos(720, yc + 150 + 30 * (i - 1), 70, 26));
    add_line(mdl, ['CtorDelay_' s '/1'], ['log_k_' lower(s) '/1']);
end

% Controller status outputs
add_block('simulink/Sinks/To Workspace', [mdl '/log_connected'], ...
    'VariableName', 'connected', 'SaveFormat', 'Timeseries', ...
    'Position', pos(640, yc + 250, 80, 26));
add_block('simulink/Sinks/To Workspace', [mdl '/log_phase'], ...
    'VariableName', 'phase', 'SaveFormat', 'Timeseries', ...
    'Position', pos(640, yc + 290, 80, 26));
add_line(mdl, 'ABT Controller/4', 'log_connected/1');
add_line(mdl, 'ABT Controller/5', 'log_phase/1');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), [mdl '.slx']));
fprintf('Built and saved %s.slx\n', mdl);
end
