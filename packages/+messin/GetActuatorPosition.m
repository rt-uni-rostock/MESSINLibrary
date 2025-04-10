function [positionSB, positionPS] = GetActuatorPosition()
    %messin.GetActuatorPosition Get the x-y-position of both actuators with respect to the body-frame.
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
    % RETURN
    % positionSB ... [m] x-y-position of the starboard actuator (from b-frame to actuator).
    % positionPS ... [m] x-y-position of the portside actuator (from b-frame to actuator).
    positionSB = [-1.3755; +0.65];
    positionPS = [-1.3755; -0.65];
end

