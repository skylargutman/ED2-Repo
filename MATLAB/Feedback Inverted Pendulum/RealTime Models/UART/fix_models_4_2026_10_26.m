% Fourth round (2026-10-26, balance pushed the cart away from the lean):
%  The port mirrored the cart but not the pendulum. Feedback's encoder card counted the cart
%  + to the left, so Counts->Meters = -.156/2048 gave model x + = right (= u + on the DAC).
%  The ESP32 counts + to the right, so the same gain made model x + = left, and the writer's
%  x(-1) kept u consistent with that, but the pendulum angle was not mirrored with them: the
%  angle-vs-cart relation was reversed from Feedback's design. Measured on the rig (logs
%  20261026_120708 and 20261026_124325, pendulum held near upright): u = -0.83*lean + 4.0*x
%  in ESP32 units, i.e. pushing away from the lean.
%  Fix: model frame = ESP32 frame = Feedback's frame (x + right, u + right):
%  Counts->Meters = +.156/2048 and the writer gain -1 -> 1. The angle conversion is unchanged.
cd('C:\Users\super\Documents\ED2'); mdl = 'SwingHoldPendulumPiMod2'; load_system(mdl);
bk = 'backup_2026-10-26_before_fixes_4';
% The two gains live in the referenced subsystem files CartDataReader.slx / CartDataWriter.slx
if ~exist(bk,'dir')
    mkdir(bk);
    for f = {[mdl '.slx'], 'CartDataReader.slx', 'CartDataWriter.slx'}, copyfile(f{1}, fullfile(bk, f{1})); end
end
cm = find_system(mdl, 'LookUnderMasks','all', 'BlockType','Gain', 'Name','Counts->Meters');
wg = find_system(mdl, 'LookUnderMasks','all', 'BlockType','Gain', 'Name','-1');
assert(numel(cm) == 1 && numel(wg) == 1, 'expected one Counts->Meters and one -1 gain');
set_param(cm{1}, 'Gain', '.156/2048');
set_param(wg{1}, 'Gain', '1');
save_system(mdl, 'SaveDirtyReferencedModels', 'on');
fprintf('%s = %s\n%s = %s\n', cm{1}, get_param(cm{1},'Gain'), wg{1}, get_param(wg{1},'Gain'));
close_system(mdl, 0);
