% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x6
%                  vectors.
%        sphereCenters -> 3xN position of centers of all spherical obstacle
%        sphereRadii -> 1xN radius of all corresponding spherical obstacles
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = checkEdgeCollision(rob,q1,q2,sphereCenters,sphereRadii)

    n = 10; % Num pts along vector to check
    m = size(q1,2); % Dimension of c-space
    
    % Generate n pts between q1 and q2
    viaPts = repmat(q2-q1,[n,1]) .* repmat(linspace(0,1,n)', [1 m]) + repmat(q1,[n,1]);
    
    % Check those pts for collision
    collision = 0;
    for i = 1:n
        collision = robotCollision(rob,viaPts(i,:),sphereCenters,sphereRadii);
        if collision
            break
        end
    end

end


