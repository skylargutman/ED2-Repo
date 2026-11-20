% Fifth round (2026-10-26, balance right-signed but far too weak):
%  The pendulum-angle PID ("PID1 Controller with D part filter") was at the Simulink block
%  defaults P=1, I=1, D=0 in every Pi version (already in the April R2025b PiMod2 model; the
%  cart PID became the PIDDP/PIDDI/PIDDD workspace variables then, the angle PID was missed).
%  Feedback's SwingHoldPendulum.mdl ships P=25 (range 10..40), I=0.2, D=1.5. On the rig
%  (log 20261026_124325, after fix 4) the angle gain fitted ~1.07 V/rad: a 7 deg lean asked
%  for 0.12 V, below breakaway, so the cart never moved.
cd('C:\Users\super\Documents\ED2'); mdl = 'SwingHoldPendulumPiMod2'; load_system(mdl);
bk = 'backup_2026-10-26_before_fixes_5';
if ~exist(bk,'dir'), mkdir(bk); copyfile([mdl '.slx'], fullfile(bk, [mdl '.slx'])); end
pid = find_system(mdl, 'LookUnderMasks','all', 'Regexp','on', 'Name','^PID1 Controller');
assert(numel(pid) == 1, 'expected one PID1 Controller block');
set_param(pid{1}, 'P', '25', 'I', '0.2', 'D', '1.5');
save_system(mdl);
fprintf('%s: P=%s I=%s D=%s N=%s\n', strrep(pid{1}, newline, ' '), get_param(pid{1},'P'), ...
    get_param(pid{1},'I'), get_param(pid{1},'D'), get_param(pid{1},'N'));
close_system(mdl, 0);
