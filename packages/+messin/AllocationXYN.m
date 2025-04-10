function [throttleSB,throttlePS,rudderSB,rudderPS] = AllocationXYN(X,Y,N)
    %messin.AllocationXYN Solve XYN allocation problem for the MESSIN USV.
    % 
    %           .-.                       .-.
    %           | |                       | |
    %           | |                       | |
    %           | |                       | |
    %           | |                       | |
    %           | |                       | |
    %           | |                       | |
    %           | |                       | |
    %           .-.                       .-.
    %          /   \                     /   \
    %         .     .                   .     .
    %         |     |                   |     |
    %         |     |                   |     |
    %       o===================================o
    %         |     |                   |     |
    %         |     |          X        |     |
    %         |     |         ^         |     |
    %         |     |         :         |     |
    %         |     |         :         |     |
    %         |     |         :       Y |     |
    %         |     |  (body) +-------> |     |
    %         |     |                   |     |
    %         |     |                   |     |
    %         |     |                   |     |
    %         |     |                   |     |
    %         |     |                   |     |
    %         |     |                   |     |
    %       o===================================o
    %         |     |                   |     |
    %         |     |                   |     |
    %         | (PS)|                   | (SB)|
    %         '.....'                   '.....'
    %            +                         +
    % 
    % PARAMETERS
    % X ... The desired force in forward direction acting on the body-frame.
    % Y ... The desired force to the right acting on the body-frame.
    % N ... The desired moment around the down axis acting on the body-frame.
    % 
    % RETURN
    % throttleSB ... Throttle of starboard actuator in range [-1,1].
    % throttlePS ... Throttle of portside actuator in range [-1,1].
    % rudderSB ... Rudder angle of starboard actuator in radians.
    % rudderPS ... Rudder angle of portside actuator in radians.

    % Actuator position
    [positionSB, positionPS] = messin.GetActuatorPosition();

    % Explicit form of pseudo-inverse matrix: invB = pinv([1 0 1 0; 0 1 0 1; -y1 x1 -y2 x2]);
    x1 = positionSB(1);
    y1 = positionSB(2);
    x2 = positionPS(1);
    y2 = positionPS(2);
    x1x1 = x1 * x1;
    x2x2 = x2 * x2;
    y1y1 = y1 * y1;
    y2y2 = y2 * y2;
    x1x2 = x1 * x2;
    y1y2 = y1 * y2;
    x1x2_2 = x1x2 + x1x2;
    y1y2_2 = y1y2 + y1y2;
    den = (x1x1-x1x2_2+x2x2+y1y1-y1y2_2+y2y2);
    frac1 = 1.0 / den;
    frac2 = 1.0 / (den + den);
    invB = [0.5 0 0; 0 0.5 0; 0.5 0 0; 0 0.5 0];
    if(abs(den) > 0.0)
        invB = [
            (x1x1-x1x2_2+x2x2+y2y2+y2y2-y1y2_2)*frac2,    ((x1+x2)*(y1-y2))*frac2,                      -(y1-y2)*frac1;
            ((y1+y2)*(x1-x2))*frac2,                      (x2x2+x2x2-x1x2_2+y1y1-y1y2_2+y2y2)*frac2,    (x1-x2)*frac1;
            (x1x1-x1x2_2+x2x2+y1y1+y1y1-2*y2*y1)*frac2,   -((x1+x2)*(y1-y2))*frac2,                     (y1-y2)*frac1;
            -((y1+y2)*(x1-x2))*frac2,                     (x1x1+x1x1-2*x2*x1+y1y1-y1y2_2+y2y2)*frac2,    -(x1-x2)*frac1
        ];
    end

    % Calculate 2D force vectors at actuator positions ([X1;Y1] for starboard actuator, [X2;Y2] for portside actuator)
    X1Y1X2Y2 = invB * [X;Y;N];
    X1 = X1Y1X2Y2(1);
    Y1 = X1Y1X2Y2(2);
    X2 = X1Y1X2Y2(3);
    Y2 = X1Y1X2Y2(4);
    thrustSB = sqrt(X1*X1 + Y1*Y1);
    thrustPS = sqrt(X2*X2 + Y2*Y2);
    angle1 = messin.SymmetricalAngle(atan2(Y1,X1));
    angle2 = messin.SymmetricalAngle(atan2(Y2,X2));

    % If thrust direction exceeds 90 degrees then invert thrust direction
    if(abs(angle1) >= deg2rad(90))
        angle1 = messin.SymmetricalAngle(angle1 + pi);
        thrustSB = -thrustSB;
    end
    if(abs(angle2) >= deg2rad(90))
        angle2 = messin.SymmetricalAngle(angle2 + pi);
        thrustPS = -thrustPS;
    end

    % angle1 and angle2 show direction of force, convert to rudder angle
    rudderSB = -angle1;
    rudderPS = -angle2;

    % convert thrust to throttle
    throttleSB = messin.ThrustToThrottle(thrustSB);
    throttlePS = messin.ThrustToThrottle(thrustPS);
end

