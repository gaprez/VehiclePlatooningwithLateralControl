function [X, Y, Yaw, VX, VY] = getReferencePathFromScenario(scenario,time)
i = 1;
X = zeros(size(time));
Y = zeros(size(time));
Yaw = zeros(size(time));

X(i) = scenario.Actors(1).Position(1);
Y(i) = scenario.Actors(1).Position(2);
Yaw(i) = scenario.Actors(1).Yaw;
VX(i) = scenario.Actors(1).Velocity(1);
VY(i) = scenario.Actors(1).Velocity(2);

yaw_flip = false;
while advance(scenario)
    i = i+1;
    X(i) = scenario.Actors(1).Position(1);
    Y(i) = scenario.Actors(1).Position(2);
    [yaw_flip, Yaw(i)] = checkYaw(scenario.Actors(1).Yaw, Yaw(i-1), yaw_flip);
    VX(i) = scenario.Actors(1).Velocity(1);
    VY(i) = scenario.Actors(1).Velocity(2);
end
X(end) = X(end-1);
Y(end) = Y(end-1);
Yaw(end) = Yaw(end-1);
VX(end) = VX(end-1);
VY(end) = VY(end-1);
end

function [yaw_flip, yaw] = checkYaw(yaw_curr, yaw_prev, yaw_flip)
    yaw = yaw_curr;
    if yaw_prev> 175 && yaw_curr < 0
        yaw_flip = true;
        yaw = yaw + 360;
    end

    if yaw_curr >175 && yaw_prev <0
        yaw_flip= false;
    end
    
end