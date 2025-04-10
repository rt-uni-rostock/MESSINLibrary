function [X,Y,N] = ActuatorsToForce(throttleStarboard, angleStarboard, throttlePortside, anglePortside)
    %messin.ActuatorsToForce Convert actuator values to a body-fixed force [X;Y;N].
    % 
    % PARAMETERS
    % throttleStarboard ... Throttle value of the starboard actuator in range [-1,+1].
    % angleStarboard    ... Angle of the starboard actuator in range [-pi/2,+pi/2].
    % throttlePortside  ... Throttle value of the portside actuator in range [-1,+1].
    % anglePortside     ... Angle of the portside actuator in range [-pi/2,+pi/2].
    % 
    % RESULT
    % X ... Longitudinal force in N.
    % X ... Lateral force in N.
    % X ... Moment in Nm.

    % Actuator position
    [positionSB, positionPS] = messin.GetActuatorPosition();

    thrustSB = messin.ThrottleToThrust(throttleStarboard);
    thrustPS = messin.ThrottleToThrust(throttlePortside);
    fx1 = thrustSB * cos(-angleStarboard);
    fy1 = thrustSB * sin(-angleStarboard);
    fx2 = thrustPS * cos(-anglePortside);
    fy2 = thrustPS * sin(-anglePortside);
    X = fx1 + fx2;
    Y = fy1 + fy2;
    N = -positionSB(2)*fx1 + positionSB(1)*fy1 - positionPS(2)*fx2 + positionPS(1)*fy2;
end

