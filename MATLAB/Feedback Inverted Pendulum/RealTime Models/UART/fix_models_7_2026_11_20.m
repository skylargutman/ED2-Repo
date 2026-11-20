% fix_models_7 (2026-11-20): start the reader's angle First Order Hold at pi (hanging), not 0.
%
% CartDataReader's "First Order Hold" (pendulum angle, Feedback frame: hanging = pi, upright = 0/2pi)
% outputs its InitialOutput on the first step, before any UART data. The default 0 means "upright".
% PendSwingUp's '<S3>/Stop Simulation' ends the model when |theta - 2pi| <= acv or theta <= acv
% (acv = 0.2), so it stopped itself on step 1: the ESP32 saw 1-2 commands and the website no data.
% The pendulum always hangs at the start of a run (the ESP32 waits for it), so pi is the right value.
% Then regenerates PendSwingUpPiMod2's code and package (deploy with tools/deploy_models_to_pi.sh).
work = 'C:\Users\super\Documents\ED2';
cd(work);
bk = fullfile(work, 'backup_2026-11-20_before_fixes_7');
if ~exist(bk, 'dir'), mkdir(bk); end
copyfile('CartDataReader.slx', bk);
copyfile('PendSwingUpPiMod2.slx', bk);

load_system('CartDataReader');
foh = 'CartDataReader/First Order Hold';
dp = get_param(foh, 'DialogParameters'); names = fieldnames(dp);
fprintf('FOH dialog params: %s\n', strjoin(names', ', '));
pname = names{contains(lower(names), 'init')};
fprintf('%s was %s\n', pname, get_param(foh, pname));
set_param(foh, pname, 'pi');
fprintf('%s now %s\n', pname, get_param(foh, pname));
fprintf('First Order Hold1 (position) %s stays %s\n', pname, get_param('CartDataReader/First Order Hold1', pname));
save_system('CartDataReader');
close_system('CartDataReader', 0);

mdl = 'PendSwingUpPiMod2';
load_system(mdl);
set_param(mdl, 'GenCodeOnly', 'on');
slbuild(mdl);
bi = load(fullfile([mdl '_ert_rtw'], 'buildInfo.mat'));
packNGo(bi.buildInfo, 'packType', 'hierarchical', 'fileName', fullfile(work, [mdl '_pkg.zip']));
close_system(mdl, 0);
ini = regexp(fileread(fullfile([mdl '_ert_rtw'], [mdl '_data.c'])), 'FirstOrderHold_IniOut[^\n]*\n[^\n]*\n[^\n]*\n\s*([^,\n]+),', 'tokens', 'once');
if ~isempty(ini), fprintf('generated FirstOrderHold_IniOut = %s\n', ini{1}); end
disp('fix_models_7 done');
