function mag = mag_open(cfg)
% mag_open — open the Phidget 1044 through MATLAB's Python interface, on its
% Spatial channel: every reading holds field, acceleration and gyro from the
% same instant, and the board's IMU filter gives pitch and roll.
%
% The reading itself is done by field_scan_tilt.py (Phidget1044Spatial, in this
% folder), the same code the Python scan uses, so both scanners read the 1044
% the same way. No Xcode/C compiler needed.
%
% The Spatial channel delivers its readings as events on a Python thread. Use
% MATLAB's out-of-process Python so that thread keeps running while MATLAB
% waits (set once, before Python is loaded in the session):
%   pyenv('ExecutionMode', 'OutOfProcess')
%
% Returns a struct: .sensor (the Python Phidget1044Spatial), .mod (the Python
% module), .info (sensor info as a MATLAB struct).

pe = pyenv;
if strlength(cfg.python) > 0
    if pe.Status == "NotLoaded"
        pyenv('Version', cfg.python, 'ExecutionMode', 'OutOfProcess');
    elseif pe.Executable ~= cfg.python
        warning("MATLAB is already using Python at %s. Restart MATLAB to switch to %s.", ...
            pe.Executable, cfg.python);
    end
elseif pe.Status == "NotLoaded" && pe.ExecutionMode ~= "OutOfProcess"
    pyenv('ExecutionMode', 'OutOfProcess');
end
pe = pyenv;
if pe.ExecutionMode ~= "OutOfProcess"
    warning("Python is running in-process. The 1044's readings may stall between calls; " + ...
        "restart MATLAB and run pyenv('ExecutionMode', 'OutOfProcess') first.");
end

% field_scan_tilt.py lives next to this file; it finds field_scan.py itself.
here = fileparts(mfilename('fullpath'));
syspath = py.sys.path;
if ~any(cellfun(@(p) strcmp(char(p), here), cell(syspath)))
    syspath.insert(int32(0), here);
end

try
    py.importlib.import_module('Phidget22.Devices.Spatial');
    mod = py.importlib.import_module('field_scan_tilt');
catch err
    pe = pyenv;
    error("Could not import Phidget22 / field_scan_tilt in Python %s (%s).\n" + ...
          "Fix: in Terminal run  ""%s"" -m pip install Phidget22\n" + ...
          "Original error: %s", pe.Version, pe.Executable, pe.Executable, err.message);
end

sensor = mod.Phidget1044Spatial(int32(cfg.mag_serial), cfg.sample_dt_s, char(cfg.algorithm));
mag.sensor = sensor;
mag.mod = mod;
mag.info = jsondecode(char(py.json.dumps(sensor.info())));
if ~strcmp(mag.info.algorithm, cfg.algorithm)
    warning("The 1044 is not running the %s filter (%s): pitch_deg/roll_deg will be NaN; " + ...
        "acc_pitch_deg/acc_roll_deg still work.", cfg.algorithm, mag.info.algorithm);
end
end
