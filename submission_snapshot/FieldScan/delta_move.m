function e = delta_move(s, cfg, xyz_probe)
% delta_move — move the PROBE to xyz_probe (mm, robot frame) with a joint
% move and block until the firmware reports the move finished.
% Returns the firmware error code: 0 ok, 1 unreachable (robot did not move),
% 5 servo pulse clamped (moved, but may not have reached the point).

c = round(xyz_probe - cfg.tcp, 2);   % firmware takes the effector centre, like the GUI
delta_send(s, '2', struct('n', 0, 'i', 0, 'v', cfg.speed_v, 'a', 0, 'c', c));

% Every move lasts at least 200 ms (T_MIN_MS), so after 100 ms any status
% line still queued from before the command can be thrown away.
pause(0.1);
flush(s, "input");

t0 = tic;
while true
    st = delta_status(s, 2);
    if isempty(st)
        error("Robot stopped sending status during a move.");
    end
    if st.en == 0
        error("Robot is disabled. Enable it before moving.");
    end
    if any(st.e == [3 4])
        error("Firmware rejected the move (e=%d).", st.e);
    end
    if st.mv == 0
        e = st.e;
        return
    end
    if toc(t0) > cfg.move_timeout_s
        error("Move to [%g %g %g] timed out.", xyz_probe);
    end
end
end
