function thrust = ThrottleToThrust(throttle)
    %messin.ThrottleToThrust Convert throttle to thrust.
    % 
    % PARAMETERS
    % throttle ... The throttle in range [-1, 1].
    % 
    % RETURN
    % thrust ... Resulting thrust force of an actuator given in N.
    speed = messin.ThrottleToSpeed(throttle);
    fPos = 0.00503145694818649 * speed.^2 + 0.0157259610543159 * speed;
    fNeg = -0.00318369940794568 * speed.^2 - 0.0118311882193987 * speed;
    mask = max(0.0, sign(speed));
    thrust = fPos .* mask + fNeg .* (1.0 - mask);
end

