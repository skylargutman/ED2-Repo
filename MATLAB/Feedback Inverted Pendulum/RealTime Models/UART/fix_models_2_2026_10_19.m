% Second round (2026-10-19, after the first motor-on run):
%  - swing-up voltages 0.15 -> 0.05 V and the first-move kick -0.5 -> -0.05 V: on this rig every
%    nonzero u also gets the ESP32's breakaway compensation (~0.7 V), so 0.15 V raced the cart into the soft limit
%  - MQTT message built with one sprintf (append/num2str generated an empty payload)
%  - stop after 40 s: the Raspberry Pi target ignores StopTime
cd('C:\Users\super\Documents\ED2'); mdl = 'SwingHoldPendulumPiMod2'; load_system(mdl);
bk = 'backup_2026-10-19_before_fixes_2';
if ~exist(bk,'dir'), mkdir(bk); copyfile([mdl '.slx'], fullfile(bk, [mdl '.slx'])); end
su = [mdl '/Swing up control'];
set_param([su '/Initial control voltage value'], 'Value', '0.05');
ex = find_system(su,'SearchDepth',1,'Regexp','on','Name','^Extra voltage'); ex = ex{1};
set_param([ex '/Constant1'], 'Value', '0.05');
set_param([su '/first cart move'], 'Before', '-0.05');
rt = sfroot; ch = rt.find('-isa','Stateflow.EMChart','Path',[mdl '/Write mqtt data/MATLAB Function']);
ch.Script = strjoin({
'function y = fmtstr(t,p,a,c)'
'% One sprintf: the append/num2str version compiled to an empty MQTT payload.'
'y = string(sprintf(''{"time": "%.3f", "position": "%.4f", "angle": "%.4f", "control": "%.3f"}'', t, p, a, c));'
'end'}, newline);
if isempty(find_system(mdl,'SearchDepth',1,'Name','run time clock'))
    add_block('simulink/Sources/Clock', [mdl '/run time clock'], 'Position', [40 520 70 550]);
    add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/stop after 40 s'], ...
        'const','40', 'relop','>=', 'Position', [110 515 170 555]);
    add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop at end of run'], 'Position', [210 520 240 550]);
    add_line(mdl, 'run time clock/1', 'stop after 40 s/1');
    add_line(mdl, 'stop after 40 s/1', 'Stop at end of run/1');
end
save_system(mdl);
fprintf('swing: init %s, extra %s, first move %s\n', get_param([su '/Initial control voltage value'],'Value'), ...
    get_param([ex '/Constant1'],'Value'), get_param([su '/first cart move'],'Before'));
set_param(mdl,'SimulationCommand','update'); fprintf('UPDATE DIAGRAM: OK\n');
set_param(mdl, 'GenCodeOnly', 'on'); slbuild(mdl);
bi = load(fullfile([mdl '_ert_rtw'], 'buildInfo.mat'));
packNGo(bi.buildInfo, 'packType','hierarchical', 'fileName', fullfile(pwd, [mdl '_pkg.zip']));
close_system(mdl, 0);   % GenCodeOnly is not saved
fprintf('CODEGEN DONE\n');
