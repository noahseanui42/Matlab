function mag = mag_open(cfg)
% mag_open — open the Phidget 1044 magnetometer channel through MATLAB's
% Python interface (Phidget22 Python package). No Xcode/C compiler needed.

pe = pyenv;
if strlength(cfg.python) > 0
    if pe.Status == "NotLoaded"
        pyenv('Version', cfg.python);
    elseif pe.Executable ~= cfg.python
        warning("MATLAB is already using Python at %s. Restart MATLAB to switch to %s.", ...
            pe.Executable, cfg.python);
    end
end

try
    mod = py.importlib.import_module('Phidget22.Devices.Magnetometer');
catch err
    pe = pyenv;
    error("Could not import Phidget22 in Python %s (%s).\n" + ...
          "Fix: in Terminal run  ""%s"" -m pip install Phidget22\n" + ...
          "Original error: %s", pe.Version, pe.Executable, pe.Executable, err.message);
end

mag = mod.Magnetometer();
if cfg.mag_serial > 0
    mag.setDeviceSerialNumber(int32(cfg.mag_serial));
end
mag.openWaitForAttachment(int32(5000));

dt_ms = max(double(mag.getMinDataInterval()), round(cfg.sample_dt_s * 1000));
mag.setDataInterval(int32(dt_ms));

% The first sample takes a moment to arrive after attach.
t0 = tic;
while true
    try
        mag.getMagneticField();
        break
    catch
        if toc(t0) > 3
            error("Magnetometer attached but sent no data within 3 s.");
        end
        pause(0.05);
    end
end
end
