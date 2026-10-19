% Set the swing-up amplitude (V) and regenerate code. Usage: amp = 0.08; run('swing_amp.m')
cd('C:\Users\super\Documents\ED2'); mdl = 'SwingHoldPendulumPiMod2'; load_system(mdl);
su = [mdl '/Swing up control'];
ex = find_system(su,'SearchDepth',1,'Regexp','on','Name','^Extra voltage'); ex = ex{1};
set_param([su '/Initial control voltage value'], 'Value', num2str(amp));
set_param([ex '/Constant1'], 'Value', num2str(amp));
set_param([su '/first cart move'], 'Before', num2str(-amp));
save_system(mdl);
fprintf('swing: init %s, extra %s, first move %s\n', get_param([su '/Initial control voltage value'],'Value'), ...
    get_param([ex '/Constant1'],'Value'), get_param([su '/first cart move'],'Before'));
set_param(mdl, 'GenCodeOnly', 'on'); slbuild(mdl);
bi = load(fullfile([mdl '_ert_rtw'], 'buildInfo.mat'));
packNGo(bi.buildInfo, 'packType','hierarchical', 'fileName', fullfile(pwd, [mdl '_pkg.zip']));
close_system(mdl, 0); fprintf('CODEGEN DONE\n');
