function throttle = SpeedToThrottle(speed)
    %messin.SpeedToThrottle Convert the motor speed to the corresponding throttle value.
    % 
    % PARAMETERS
    % speed ... The motor speed in rad/s.
    % 
    % RETURN
    % throttle ... The corresponding throttle in range [-1, 1].
    rpm = speed * (30.0 / pi);
    eot = rpm / 1309.64608792645;
    throttle = min(max(eot / 0.7, -1.0), 1.0);
end

