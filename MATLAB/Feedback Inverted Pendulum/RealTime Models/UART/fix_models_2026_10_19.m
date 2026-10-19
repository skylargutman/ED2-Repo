% Fix Alex's UART model files for the single-board ESP32 (spec 2026-10-10, review 2026-10-19).
cd('C:\Users\super\Documents\ED2');
bk = 'backup_2026-10-19_before_fixes';
if ~exist(bk, 'dir'), mkdir(bk); end
for f = {'SwingHoldPendulumPiMod2.slx','CartDataReader.slx','CartDataWriter.slx'}
    if ~exist(fullfile(bk, f{1}), 'file'), copyfile(f{1}, fullfile(bk, f{1})); end
end
fprintf('backups in %s\n', fullfile(pwd, bk));

%% ---------------- CartDataWriter ----------------
w = 'CartDataWriter'; load_system(w);
neg = find_system(w, 'SearchDepth',1, 'BlockType','Gain', 'Gain','-1'); neg = neg{1};
% 1. drop the old DAC convention: 2.5 V offset, Gain1, 0..5 V saturation
% 3. drop the model's friction compensation (the ESP32's motorVolts compensates)
for b = {'Constant','Add','Gain1','Saturation','Friction Compensation','Add1'}
    delete_block([w '/' b{1}]);
end
L = find_system(w, 'FindAll','on', 'SearchDepth',1, 'Type','line');
for i = 1:numel(L)
    if ishandle(L(i)) && (get_param(L(i),'SrcBlockHandle') == -1 || any(get_param(L(i),'DstBlockHandle') == -1)), delete_line(L(i)); end
end
add_line(w, 'In1/1', [get_param(neg,'Name') '/1'], 'autorouting','smart');
add_line(w, 'Saturation1/1', 'Encode Command Function/1', 'autorouting','smart');
set_param([w '/Saturation1'], 'UpperLimit','2.5', 'LowerLimit','-2.5');
save_system(w);
fprintf('writer: In1 -> %s -> Saturation1(+-2.5 V) -> Encode -> UART @ %s\n', get_param(neg,'Name'), get_param([w '/UART Serial Write'],'Baudrate_'));

%% ---------------- CartDataReader ----------------
r = 'CartDataReader'; load_system(r);
% 4. pendulum encoder is 1968 counts/rev (measured), hanging = half a turn
set_param([r '/Counts->Radians'], 'Gain', '2*pi/1968');
set_param([r '/Constant2'], 'Value', '-984');
% 6. old system-ready pulse on Pi GPIO 6 (pin 31): not wired on the single-board rig
for b = {'reset','DO Generate1','Clock'}
    delete_block([r '/' b{1}]);
end
L = find_system(r, 'FindAll','on', 'SearchDepth',1, 'Type','line');
for i = 1:numel(L)
    if ishandle(L(i)) && (get_param(L(i),'SrcBlockHandle') == -1 || any(get_param(L(i),'DstBlockHandle') == -1)), delete_line(L(i)); end
end
% 5. hold the last valid packet instead of passing on a misaligned or missing one
rt = sfroot; ch = rt.find('-isa','Stateflow.EMChart', 'Path', [r '/Encoder_data/MATLAB Function']);
ch.Script = strjoin({ ...
'function [cart, pend, flags, valid] = decode_packet(raw_bytes)'
'% 12-byte measurement from the ESP32: A5 5A, cart int32 LE, pend int32 LE, flags, XOR(bytes 3..11).'
'% A missing, short or misaligned read (valid = false) keeps the last good values, so a lost'
'% byte never feeds garbage counts to the controller.'
'persistent last_cart last_pend last_flags'
'if isempty(last_cart)'
'    last_cart = 0; last_pend = 0; last_flags = 0;'
'end'
'b = uint8(raw_bytes(1:12));'
'header_ok = (b(1) == 165) && (b(2) == 90);'
'xor_val = uint8(0);'
'for i = 3:11'
'    xor_val = bitxor(xor_val, b(i));'
'end'
'valid = boolean(header_ok && (xor_val == b(12)));'
'if valid'
'    cart_i32 = typecast(b(3:6), ''int32'');'
'    pend_i32 = typecast(b(7:10), ''int32'');'
'    last_cart  = double(cart_i32(1));'
'    last_pend  = double(pend_i32(1));'
'    last_flags = double(b(11));'
'end'
'cart  = last_cart;'
'pend  = last_pend;'
'flags = last_flags;'
'end'}, newline);
save_system(r);
fprintf('reader: rad gain %s, offset %s, serial %s @ %s, %d bytes\n', get_param([r '/Counts->Radians'],'Gain'), ...
    get_param([r '/Constant2'],'Value'), get_param([r '/Encoder_data/UART Serial Read'],'SCIModule_'), ...
    get_param([r '/Encoder_data/UART Serial Read'],'Baudrate_'), get_param([r '/Encoder_data/UART Serial Read'],'DataLength'));
