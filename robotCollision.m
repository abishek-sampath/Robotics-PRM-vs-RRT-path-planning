% 
% Evaluate whether the configuration <q> is in collision with a spherical
% obstacle centered at <sphereCenter> with radius <r>.
% 
% input: rob -> a puma 560 robot
%        q -> 1x6 vector of joint angles
%        sphereCenters -> 3xN position of centers of all spherical obstacle
%        sphereRadii -> 1xN radius of all corresponding spherical obstacles
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = robotCollision(rob,q,sphereCenters,sphereRadii)

    numObstacles = length(sphereRadii);
    for i = 1 : numObstacles
        collision = robotCollisionInSphere(rob, q, sphereCenters(:, i), sphereRadii(i));
        if collision == 1
            break
        end
    end
end

% check collision in single sphere obstacle
function collision = robotCollisionInSphere(rob,qIn,sphereCenter,r)
    q = qIn';   % qIn is a row vector. convert to column vector.
    x1 = [0;0;0];
    T2 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q);
    x2 = T2(1:3,4);
    T3 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q);
    x3 = T3(1:3,4);
    T4 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q) * rob.A(5,q);
    x4 = T4(1:3,4);
    T5 = rob.A(1,q) * rob.A(2,q) * rob.A(3,q) * rob.A(4,q) * rob.A(5,q) * rob.A(6,q);
    x5 = T5(1:3,4);
    
    vec = 0:0.1:1;
    m = size(vec,2);
    
    x12 = repmat(x2-x1,1,m) .* repmat(vec,3,1) + repmat(x1,1,m);
    x23 = repmat(x3-x2,1,m) .* repmat(vec,3,1) + repmat(x2,1,m);
    x34 = repmat(x4-x3,1,m) .* repmat(vec,3,1) + repmat(x3,1,m);
    x45 = repmat(x5-x4,1,m) .* repmat(vec,3,1) + repmat(x4,1,m);
    x = [x12 x23 x34 x45];
    
    if sum(sum((x - repmat(sphereCenter,1,size(x,2))).^2,1) < r^2) > 0
        collision = 1;
    else
        collision = 0;
    end

end






