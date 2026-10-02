function s = delta_connect(cfg)
% delta_connect — open the serial link to the delta_servo firmware and
% wait for its 20 ms status stream. Close the Python GUI first: only one
% program can hold the port.

s = serialport(cfg.port, cfg.baud, "Timeout", 1);
configureTerminator(s, "CR/LF");
setDTR(s, true);   % R4 USB CDC sends nothing until DTR is asserted
pause(1);
flush(s, "input");

if isempty(delta_status(s, 3))
    error("Connected to %s but no status lines arrived. Is delta_servo flashed?", cfg.port);
end
end
