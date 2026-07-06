function run_scenarios()
%RUN_SCENARIOS  Build (if needed) and run all ABT scenarios in Simulink.
%
%   Each scenario scripts the EMF targets of the three sources as
%   timeseries (a generator failure is a collapse of its target to 0; the
%   model applies the field time constant) plus the step-load command,
%   then simulates and saves a three-panel plot to results/matlab/.
%
%   Scenarios mirror python/run_scenarios.py (S1-S4).  The make-before-
%   break back-feed demo (S5) is Python-only: it requires swapping in a
%   deliberately wrong controller, which would obscure this model.

here = fileparts(mfilename('fullpath'));
addpath(here);
outdir = fullfile(here, '..', 'results', 'matlab');
if ~exist(outdir, 'dir'), mkdir(outdir); end

P   = abt_params();
mdl = 'abt_model';
if ~bdIsLoaded(mdl)
    if exist(fullfile(here, [mdl '.slx']), 'file')
        load_system(fullfile(here, [mdl '.slx']));
    else
        build_abt_model();
    end
end

E = P.emf_set;

% name, t_end, [t_fail t_recover] per source (NaN = never), step-load time
sc = {
 's1_single_failure',       3.0, [1.0 NaN], [NaN NaN], [NaN NaN], NaN
 's2_cascading_failure',    5.0, [1.0 NaN], [3.0 NaN], [NaN NaN], NaN
 's3_recovery_retransfer',  6.0, [1.0 3.0], [NaN NaN], [NaN NaN], NaN
 's4_transient_ride_through',3.0,[1.0 1.03],[NaN NaN], [NaN NaN], 2.0
};

for k = 1:size(sc, 1)
    [name, t_end] = deal(sc{k, 1}, sc{k, 2});
    tv = (0:P.ts:t_end)';

    for i = 1:3
        fr  = sc{k, 2 + i};                       % [t_fail t_recover]
        emf = E(i) * ones(size(tv));
        if ~isnan(fr(1))
            failed = tv >= fr(1);
            if ~isnan(fr(2)), failed = failed & tv < fr(2); end
            emf(failed) = 0;
        end
        ts = timeseries(emf, tv);
        switch i
            case 1, assignin('base', 'emf_main', ts);
            case 2, assignin('base', 'emf_apu',  ts);
            case 3, assignin('base', 'emf_batt', ts);
        end
    end
    tls = sc{k, 6};
    step = double(~isnan(tls) & tv >= tls);
    assignin('base', 'load_step', timeseries(step, tv));

    fprintf('Running %s ...\n', name);
    out = sim(mdl, 'StopTime', num2str(t_end), ...
        'ReturnWorkspaceOutputs', 'on');

    plot_scenario(out, name, outdir);
end
end

function plot_scenario(out, name, outdir)
% Three panels: voltages / contactor states / source currents.
% Colors match the Python reference plots.
cM = [42 120 214] / 255;  cA = [27 175 122] / 255;  cB = [237 161 0] / 255;
ink = [11 11 11] / 255;

g   = @(v) out.(v);   % To Workspace timeseries by name
f   = figure('Visible', 'off', 'Position', [100 100 900 700], ...
             'Color', [0.988 0.988 0.984]);
tl  = tiledlayout(f, 3, 1, 'TileSpacing', 'compact');
title(tl, strrep(name, '_', '\_'), 'FontWeight', 'bold');

ax1 = nexttile;  hold(ax1, 'on');
plot(ax1, g('v_main').Time, g('v_main').Data, 'Color', cM, 'LineWidth', 1.2);
plot(ax1, g('v_apu').Time,  g('v_apu').Data,  'Color', cA, 'LineWidth', 1.2);
plot(ax1, g('v_batt').Time, g('v_batt').Data, 'Color', cB, 'LineWidth', 1.2);
plot(ax1, g('v_bus').Time,  g('v_bus').Data,  'Color', ink, 'LineWidth', 1.8);
yline(ax1, 18, '--', 'undervoltage threshold', 'Color', [0.54 0.53 0.51]);
ylabel(ax1, 'Voltage [V]');  ylim(ax1, [-1.5 32]);  grid(ax1, 'on');
legend(ax1, {'MAIN terminal', 'APU terminal', 'BATT terminal', 'Bus'}, ...
       'Location', 'east');

ax2 = nexttile;  hold(ax2, 'on');
stairs(ax2, g('k_main').Time, 2.6 + g('k_main').Data, 'Color', cM, 'LineWidth', 1.4);
stairs(ax2, g('k_apu').Time,  1.3 + g('k_apu').Data,  'Color', cA, 'LineWidth', 1.4);
stairs(ax2, g('k_batt').Time,       g('k_batt').Data, 'Color', cB, 'LineWidth', 1.4);
yticks(ax2, [0.5 1.8 3.1]);  yticklabels(ax2, {'K\_BATT', 'K\_APU', 'K\_MAIN'});
ylim(ax2, [-0.3 4]);  grid(ax2, 'on');
ylabel(ax2, 'Contactors');

ax3 = nexttile;  hold(ax3, 'on');
plot(ax3, g('i_main').Time, g('i_main').Data, 'Color', cM, 'LineWidth', 1.2);
plot(ax3, g('i_apu').Time,  g('i_apu').Data,  'Color', cA, 'LineWidth', 1.2);
plot(ax3, g('i_batt').Time, g('i_batt').Data, 'Color', cB, 'LineWidth', 1.2);
yline(ax3, 0, 'Color', [0.76 0.76 0.72]);
ylabel(ax3, 'Source current [A]');  xlabel(ax3, 'Time [s]');
ylim(ax3, [-5 45]);  grid(ax3, 'on');   % recharge inrush peaks are clipped
legend(ax3, {'MAIN', 'APU', 'BATT'}, 'Location', 'northeast');

linkaxes([ax1 ax2 ax3], 'x');
exportgraphics(f, fullfile(outdir, [name '.png']), 'Resolution', 150);
close(f);
fprintf('  wrote %s.png\n', fullfile(outdir, name));
end
