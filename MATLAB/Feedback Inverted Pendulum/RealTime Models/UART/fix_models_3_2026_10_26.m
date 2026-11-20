% Third round (2026-10-26, swing-up buzzed in place):
%  - The swing-up direction is the sign of the filtered pendulum velocity. Hanging still, the
%    angle flickers by one encoder count (3.2 mrad) and the 16 Hz velocity filter rings on every
%    flicker, so the sign flipped ~30 times/s and the cart buzzed +-0.3 mm (log 20261026_120708_COM3).
%    A relay with +-0.3 rad/s hysteresis now decides the sign: a one-count flicker peaks at
%    ~0.2 rad/s, a 6 deg swing at ~0.56 rad/s.
%  - first cart move -0.08 -> -0.3 V: -0.08 was cancelled by the +0.08 V swing term, so the
%    pendulum never got a starting swing. With the hysteresis, the swing must come from the kick.
cd('C:\Users\super\Documents\ED2'); mdl = 'SwingHoldPendulumPiMod2'; load_system(mdl);
bk = 'backup_2026-10-26_before_fixes_3';
if ~exist(bk,'dir'), mkdir(bk); copyfile([mdl '.slx'], fullfile(bk, [mdl '.slx'])); end
su = [mdl '/Swing up control'];
vs = [su '/Voltage sign switching depending on pendulum position'];
rl = [vs '/velocity sign hysteresis'];
if isempty(find_system(vs,'SearchDepth',1,'Name','velocity sign hysteresis'))
    delete_line(vs, 'angular velocity/1', 'zero velocity/2');
    pos = get_param([vs '/zero velocity'], 'Position');
    add_block('simulink/Discontinuities/Relay', rl, 'Position', [pos(1)-110 pos(4)+10 pos(1)-60 pos(4)+40]);
    add_line(vs, 'angular velocity/1', 'velocity sign hysteresis/1', 'autorouting', 'on');
    add_line(vs, 'velocity sign hysteresis/1', 'zero velocity/2', 'autorouting', 'on');
end
set_param(rl, 'OnSwitchValue', '0.3', 'OffSwitchValue', '-0.3', 'OnOutputValue', '1', 'OffOutputValue', '-1');
set_param([su '/first cart move'], 'Before', '-0.3');
save_system(mdl);
fprintf('relay on %s off %s, zero velocity thr %s, first move %s\n', get_param(rl,'OnSwitchValue'), ...
    get_param(rl,'OffSwitchValue'), get_param([vs '/zero velocity'],'Threshold'), get_param([su '/first cart move'],'Before'));
set_param(mdl,'SimulationCommand','update'); fprintf('UPDATE DIAGRAM: OK\n');
set_param(mdl, 'GenCodeOnly', 'on'); slbuild(mdl);
bi = load(fullfile([mdl '_ert_rtw'], 'buildInfo.mat'));
packNGo(bi.buildInfo, 'packType','hierarchical', 'fileName', fullfile(pwd, [mdl '_pkg.zip']));
close_system(mdl, 0);   % GenCodeOnly is not saved
fprintf('CODEGEN DONE\n');
