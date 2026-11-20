% Port the DAC-era PiMod2 models to the single-board UART rig (2026-11-06), the same way
% SwingHoldPendulumPiMod2 was ported by hand (fix_models_* in this folder):
%  1. "Cart Position_Pendulum Angle*" (SPI encoder card + GPIO 6 reset) -> CartDataReader
%     "Feedback DAC*" (I2C MCP4725, 2.5 V offset, friction compensation) -> CartDataWriter
%     Both are the shared subsystem files next to the models; they present Feedback's frame
%     (x + right, u + right, hanging = pi, 1968 counts/rev), so the controllers are unchanged.
%  2. Telemetry: one sprintf (the string-append version compiles to an empty MQTT payload on
%     R2026b), the model's own field names, the angle wrapped to (-pi, pi] (0 = upright), and
%     control/voltage = the command actually sent to the ESP32 (CartDataWriter's u_sent).
%  3. Stop after the model's StopTime: the Raspberry Pi target ignores StopTime.
% Reads the PiMod2 originals from the folder above (unchanged) and writes <Name>PiMod2.slx into
% the working folder C:\Users\super\Documents\ED2, where CartDataReader/Writer live.
% SwingHoldPendulumPiMod2 is not touched: it was ported by hand and has its own fixes.

src = fullfile(fileparts(mfilename('fullpath')), '..');
work = 'C:\Users\super\Documents\ED2';
models = {'CartControl','CartIdent','CraneIdent','CraneStab','InvPendIdent','PendStabPD', ...
          'PendSwingUp','PendulumFriction','PendulumTest','SwingHoldPendulumExtra','UpDownPendulum'};
if exist('only', 'var'), models = only; end   % e.g. only = {'PendulumTest'}; run(...)
cd(work);
report = {};
for i = 1:numel(models)
    mdl = [models{i} 'PiMod2'];
    try
        if bdIsLoaded(mdl), close_system(mdl, 0); end
        copyfile(fullfile(src, [mdl '.slx']), fullfile(work, [mdl '.slx']), 'f');
        fileattrib(fullfile(work, [mdl '.slx']), '+w');
        load_system(mdl);
        notes = port_one(mdl);
        set_param(mdl, 'SimulationCommand', 'update');   % compiles the diagram
        save_system(mdl);
        close_system(mdl, 0);
        report{end+1} = sprintf('OK    %-30s %s', mdl, notes); %#ok<SAGROW>
    catch e
        if bdIsLoaded(mdl), close_system(mdl, 0); end
        report{end+1} = sprintf('FAIL  %-30s %s', mdl, e.message); %#ok<SAGROW>
    end
end
fprintf('\n%s\n', strjoin(report, newline));

function notes = port_one(mdl)
    notes = '';
    % 1. I/O subsystems -> shared UART subsystems
    rd = only_block(mdl, '^Cart Position_Pendulum Angle');
    swap(mdl, rd, 'CartDataReader');
    dac = find_system(mdl, 'SearchDepth',1, 'Regexp','on', 'Name','^Feedback DAC');
    if ~isempty(dac), swap(mdl, dac{1}, 'CartDataWriter'); dac = dac{1}; else, dac = ''; notes = 'no motor output;'; end
    left = find_system(mdl, 'LookUnderMasks','all', 'BlockType','MATLABSystem', 'Regexp','on', ...
        'System','RaspiGPIOWrite|I2CMasterWrite|SPIMasterTransfer');
    assert(isempty(left), 'old hardware blocks left: %s', strjoin(left, ', '));

    % 2. Telemetry
    mq = find_system(mdl, 'SearchDepth',1, 'Regexp','on', 'Name','^Write mqtt data');
    if isempty(mq)
        notes = [notes ' no telemetry;'];
    else
        mq = mq{1};
        ch = sfroot().find('-isa','Stateflow.EMChart', 'Path',[mq '/MATLAB Function']);
        old = ch.Script;
        args = regexp(old, 'function\s+y\s*=\s*\w+\(([^)]*)\)', 'tokens', 'once');
        args = strtrim(strsplit(args{1}, ','));
        keys = regexp(old, '(\w+):\s*"\s*\+\s*string\((\w+)\)', 'tokens');
        assert(numel(keys) == numel(args), 'could not read the telemetry fields of: %s', old);
        fmt = {}; vals = {}; body = {};
        for k = 1:numel(keys)
            key = keys{k}{1}; v = keys{k}{2};
            if strcmp(key, 'time'), f = '%.3f'; else, f = '%.4f'; end
            fmt{end+1} = sprintf('"%s": "%s"', key, f); vals{end+1} = v; %#ok<AGROW>
            if strcmp(key, 'angle')
                body{end+1} = sprintf('%s = -(mod(-%s + pi, 2*pi) - pi);   %% (-pi, pi]: 0 = upright', v, v); %#ok<AGROW>
            end
            if any(strcmp(key, {'control','voltage'})) && ~isempty(dac)
                % input k-1 of the block (the first argument is the internal clock)
                ph = get_param(mq, 'PortHandles'); l = get_param(ph.Inport(k-1), 'Line');
                if l > 0, delete_line(l); end
                add_line(mdl, [get_param(dac,'Name') '/1'], [get_param(mq,'Name') '/' num2str(k-1)], 'autorouting','on');
                notes = [notes ' ' key ' = u_sent;']; %#ok<AGROW>
            end
        end
        ch.Script = strjoin([{sprintf('function y = fmtstr(%s)', strjoin(args, ','))}, ...
            {'% One sprintf: the string-append version compiled to an empty MQTT payload on R2026b.'}, body, ...
            {sprintf('y = string(sprintf(''{%s}'', %s));', strjoin(fmt, ', '), strjoin(vals, ', '))}, {'end'}], newline);
        mws = get_param(mdl, 'ModelWorkspace');
        if ~hasVariable(mws, 'MQTTsample'), assignin(mws, 'MQTTsample', 0.1); end
    end

    % 3. Stop at the end of the run
    T = get_param(mdl, 'StopTime');
    if isempty(find_system(mdl, 'SearchDepth',1, 'Name','Stop at end of run'))
        add_block('simulink/Sources/Clock', [mdl '/run time clock'], 'Position',[40 620 70 650]);
        add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/stop after StopTime'], ...
            'const',T, 'relop','>=', 'Position',[110 615 170 655]);
        add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop at end of run'], 'Position',[210 620 240 650]);
        add_line(mdl, 'run time clock/1', 'stop after StopTime/1');
        add_line(mdl, 'stop after StopTime/1', 'Stop at end of run/1');
    end
    notes = sprintf('%s stop %s s', notes, T);
end

function b = only_block(mdl, pat)
    b = find_system(mdl, 'SearchDepth',1, 'Regexp','on', 'Name',pat);
    assert(numel(b) == 1, 'expected one block matching %s', pat);
    b = b{1};
end

function swap(mdl, blk, ref)
    % Replace subsystem blk by a Subsystem Reference to ref, keeping its name and wiring.
    name = get_param(blk, 'Name'); pos = get_param(blk, 'Position'); ph = get_param(blk, 'PortHandles');
    ins = {}; outs = {};
    for k = 1:numel(ph.Inport)
        l = get_param(ph.Inport(k), 'Line');
        if l > 0, ins{k} = get_param(l, 'SrcPortHandle'); delete_line(l); end %#ok<AGROW>
    end
    for k = 1:numel(ph.Outport)
        l = get_param(ph.Outport(k), 'Line');
        if l > 0, outs{k} = get_param(l, 'DstPortHandle'); delete_line(l); end %#ok<AGROW>
    end
    delete_block(blk);
    new = add_block('simulink/Ports & Subsystems/Subsystem Reference', [mdl '/' name], ...
        'ReferencedSubsystem', ref, 'Position', pos);
    nph = get_param(new, 'PortHandles');
    assert(numel(nph.Inport) >= numel(ph.Inport) && numel(nph.Outport) >= numel(ph.Outport), ...
        '%s has fewer ports than %s', ref, name);
    for k = 1:numel(ins),  if ~isempty(ins{k}), add_line(mdl, ins{k}, nph.Inport(k), 'autorouting','on'); end, end
    for k = 1:numel(outs), for d = outs{k}(:)', add_line(mdl, nph.Outport(k), d, 'autorouting','on'); end, end
end
