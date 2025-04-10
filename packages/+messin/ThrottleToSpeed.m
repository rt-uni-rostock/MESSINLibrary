function speed = ThrottleToSpeed(throttle)
    %messin.ThrottleToSpeed Convert the throttle value to the corresponding motor speed.
    % 
    % PARAMETERS
    % throttle ... The throttle in range [-1, 1].
    % 
    % RETURN
    % speed ... The corresponding speed in rad/s.
    eot = 0.7 * min(max(throttle, -1.0), 1.0);
    rpm = 1309.64608792645 * eot;
    speed = (pi / 30.0) * rpm;
end

