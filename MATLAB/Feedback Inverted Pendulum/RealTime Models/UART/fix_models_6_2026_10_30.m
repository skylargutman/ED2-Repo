% Sixth round (2026-10-30, website telemetry showed meaningless values):
%  "control" was wired from the balance controller's output, ahead of the stabilization/swing-up
%  switch, so whenever the pendulum was outside the balance window the website got the controller's
%  unused, wound-up output (549.8 on a hanging pendulum with the motor off, run of 2026-10-30
%  13:03). "angle" was the unwrapped pend*k + pi, so a pendulum that fell over read 9.5 rad.
%  Fix: CartDataWriter gets an output "u_sent" (after its gain and the +-2.5 V clamp, exactly what
%  goes to the ESP32) that feeds the telemetry's control input, and the telemetry function wraps
%  the angle to (-pi, pi]: 0 = upright, +-pi = hanging. The controllers are unchanged.
cd('C:\Users\super\Documents\ED2'); mdl = 'SwingHoldPendulumPiMod2';
bk = 'backup_2026-10-30_before_fixes_6';
if ~exist(bk,'dir')
    mkdir(bk);
    for f = {[mdl '.slx'], 'CartDataReader.slx', 'CartDataWriter.slx'}, copyfile(f{1}, fullfile(bk, f{1})); end
end

% 1. CartDataWriter (a subsystem file): branch the clamped command to a new output port
wr = 'CartDataWriter'; load_system(wr);
if isempty(find_system(wr, 'SearchDepth',1, 'BlockType','Outport', 'Name','u_sent'))
    sat = find_system(wr, 'SearchDepth',1, 'BlockType','Saturate'); assert(numel(sat) == 1);
    p = get_param(sat{1}, 'Position');
    add_block('simulink/Sinks/Out1', [wr '/u_sent'], 'Position', [p(3)+60, p(4)+40, p(3)+90, p(4)+54]);
    add_line(wr, [get_param(sat{1},'Name') '/1'], 'u_sent/1', 'autorouting','on');
end
save_system(wr); close_system(wr);

% 2. Telemetry "control" from the writer's u_sent instead of the stabilization controller
load_system(mdl);
w = [mdl '/Write mqtt data']; dac = [mdl '/Feedback DAC'];
ph = get_param(w, 'PortHandles'); l = get_param(ph.Inport(3), 'Line');
if l > 0, delete_line(l); end
add_line(mdl, 'Feedback DAC/1', 'Write mqtt data/3', 'autorouting','on');

% 3. Wrap the angle in the telemetry function
ch = sfroot().find('-isa','Stateflow.EMChart', 'Path',[w '/MATLAB Function']); assert(numel(ch) == 1);
ch.Script = sprintf([ ...
    'function y = fmtstr(t,p,a,c)\n' ...
    '%% One sprintf: the append/num2str version compiled to an empty MQTT payload.\n' ...
    '%% angle wrapped to (-pi, pi]: 0 = upright, +-pi = hanging. c is the command sent to the ESP32.\n' ...
    'a = -(mod(-a + pi, 2*pi) - pi);\n' ...
    'y = string(sprintf(''{"time": "%%.3f", "position": "%%.4f", "angle": "%%.4f", "control": "%%.3f"}'', t, p, a, c));\n' ...
    'end\n']);
save_system(mdl, 'SaveDirtyReferencedModels','on');

% Report
ph = get_param(w, 'PortHandles');
for k = 1:3
    l = get_param(ph.Inport(k), 'Line'); s = get_param(l, 'SrcPortHandle');
    fprintf('telemetry in%d <- %s port %d\n', k, strrep(get_param(s,'Parent'),newline,' '), get_param(s,'PortNumber'));
end
fprintf('%s\n', ch.Script);
close_system(mdl, 0);
