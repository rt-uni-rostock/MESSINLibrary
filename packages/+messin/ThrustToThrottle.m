function throttle = ThrustToThrottle(thrust)
    %messin.ThrustToThrottle Convert thrust to throttle.
    % 
    % PARAMETERS
    % thrust ... Thrust force of an actuator given in N.
    % 
    % RETURN
    % throttle ... The corresponding throttle in range [-1, 1].
    speed = 0.0;
    if(thrust > 0.0)
        aPos = 0.00503145694818649;
        bPos = 0.0157259610543159;
        speed = (-bPos + sqrt(bPos*bPos + 4.0*aPos*thrust)) / (2.0 * aPos);
    elseif(thrust < 0.0)
        aNeg = -0.00318369940794568;
        bNeg = -0.0118311882193987;
        speed = (-bNeg + sqrt(bNeg*bNeg + 4.0*aNeg*thrust)) / (2.0 * aNeg);
    end
    throttle = messin.SpeedToThrottle(speed);
end

