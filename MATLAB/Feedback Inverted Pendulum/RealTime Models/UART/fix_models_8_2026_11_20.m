% fix_models_8 (2026-11-20): SwingHoldPendulumExtra swing-up push 0.15 V -> 0.5 V.
%
% Its swing-up is  u = Constant1*sin(angle)*Gain1 + "Initial control voltage value"  (sign from the
% pendulum velocity), plus the cart-position PID. At 0.15 V the command stayed within +-0.15 V, which
% with the ESP32's friction offsets (FRIC_POS_V 0.39 / FRIC_NEG_V 0.30) is below breakaway: the cart
% never moved (rig runs 2026-11-20). PendSwingUp swings up with ua = 0.5 V, so the base push is set to
% 0.5; the sin term (Constant1 = 0.15) is left as Feedback had it. Output saturation is +-2.5 V.
% Regenerating also picks up CartDataReader's angle First Order Hold starting at pi (fix_models_7).
work = 'C:\Users\super\Documents\ED2';
cd(work);
mdl = 'SwingHoldPendulumExtraPiMod2';
bk = fullfile(work, 'backup_2026-11-20_before_fixes_8');
if ~exist(bk, 'dir'), mkdir(bk); end
copyfile([mdl '.slx'], bk);

load_system(mdl);
b = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Name', 'Initial control voltage value');
assert(numel(b) == 1, 'expected one "Initial control voltage value" block, found %d', numel(b));
fprintf('%s: %s -> 0.5\n', b{1}, get_param(b{1}, 'Value'));
set_param(b{1}, 'Value', '0.5');
save_system(mdl);

set_param(mdl, 'GenCodeOnly', 'on');
slbuild(mdl);
bi = load(fullfile([mdl '_ert_rtw'], 'buildInfo.mat'));
packNGo(bi.buildInfo, 'packType', 'hierarchical', 'fileName', fullfile(work, [mdl '_pkg.zip']));
close_system(mdl, 0);
disp('fix_models_8 done');
