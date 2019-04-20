% run startup_rvc FIRST before running this function.
% SYNTAX: start_path_planning(algorithmType,startX, startY, startZ, endX, endY, endZ)
%   Required: algorithmType 
%               ('PRM' or 'RRT')
%   Optional: startX, startY, startZ, endX, endY, endZ 
%               (Default coordinates will be used)
function start_path_planning(algorithmType, startX, startY, startZ, endX, endY, endZ)

    close all;
    
    if nargin < 1
        error("Error: please enter arguments\n%s \n\n\t%s \n\t%s",...
                sprintf("SYNTAX: \n\tstart_path_planning(algorithmType,startX, startY, startZ, endX, endY, endZ"),...
                sprintf("Required: \n\t\talgorithmType \n\t\t('PRM' or 'RRT')"),...
                sprintf("Optional: \n\t\tstartX, startY, startZ, endX, endY, endZ) \n\t\t(Default coordinates will be used)"));
    end
    
    % Initialize and Create Puma 560 Robot
    mdl_puma560;
	rob = p560;

    % Configure start/goal configuration; min/max c-space bounds
    if ~exist('startX','var') || ~exist('startY','var') || ~exist('startZ','var')
        %startCoord = [-0.75; 0.0; 0.0];
        startCoord = [0.5; -0.5; 0.5];
    else
        startCoord = [startX; startY; startZ];
    end
    
    if ~exist('endX','var') || ~exist('endY','var') || ~exist('endZ','var')
        %endCoord = [0.5; 0.5; 0.5];
        endCoord = [-0.5; 0.4; -0.5];
    else
        endCoord = [endX; endY; endZ];
    end
    
    qStart = rob.ikine6s(transl(startCoord));
    qGoal = rob.ikine6s(transl(endCoord));

    
    % Set up obstacles
    sphereCenter0 = [0.5;0.0;0];
    sphereRadius0 = 0.2;
    sphereCenter1 = [0.0; 0.5; 0.0];
    sphereRadius1 = 0.1;
    sphereCenter2 = [-0.75; -0.25; 0.20];
    sphereRadius2 = 0.2;
    sphereCenter3 = [0.0; 0.75; -0.5];
    sphereRadius3 = 0.3;
    sphereCenter4 = [-0.5; -0.5; -0.5];
    sphereRadius4 = 0.2;
    % arrange sphere centers as a 3 x n matrix
    sphereCenters = [sphereCenter0, sphereCenter1, sphereCenter2, sphereCenter3, sphereCenter4];
    % arrange sphere radii as a 1 x n matrix
    sphereRadii = [sphereRadius0, sphereRadius1, sphereRadius2, sphereRadius3, sphereRadius4];
    
    % Plot robot and sphere
    rob.plot(qStart,'trail', 'ro');
    hold on;	
    for i = 1 : length(sphereRadii)
        drawSphere(sphereCenters(:, i), sphereRadii(i));
    end

    %check to see if start or goal configurations collide with obstacles
    if robotCollision(rob,qStart,sphereCenters,sphereRadii) == 1 ...
        || robotCollision(rob,qGoal,sphereCenters,sphereRadii) == 1
        error("Error: start/goal configuration is in collision with obstacle(s)");
    end
    
    %run the algorithm to get the path milestones
    if algorithmType == "RRT"
        qMilestones = rrt_algorithm(rob,qStart,qGoal,sphereCenters,sphereRadii);
        fprintf("Completed RRT\n");
    elseif algorithmType == "PRM"
        qMilestones = prm_algorithm(rob,qStart,qGoal,sphereCenters,sphereRadii);
        fprintf("Completed PRM\n");
    else
        error("Error: Algorithm type not yet supported. Currently supported algorithms as 'PRM' and 'RRT'");        
    end
   
    if qMilestones
        % Plot robot following path
        qTrajectory = interpolateMilestones(qMilestones);
        rob.plot(qTrajectory, 'trail', 'ro');
    else
        % no milestones are found 
        fprintf('No valid path found from start to goal\n');
    end
    
end


% create a trajectory of the path from the milestones
% input: qMilestones -> mx4 matrix of vertices along path from start to
%                        goal.
function trajectory = interpolateMilestones(qMilestones)
    d = 0.05;
    trajectory = [];

    for i = 2 : size(qMilestones, 1)        
        delta = qMilestones(i, :) - qMilestones(i - 1, :);
        m = max(floor(norm(delta) / d), 1);
        vec = linspace(0, 1, m);
        leg = repmat(delta', 1, m) .* repmat(vec, size(delta, 2), 1) + repmat(qMilestones(i - 1, :)', 1, m);
        trajectory = [trajectory; leg'];
    end
end



% Draws a sphere on the plot with given radius at the specified location
% input: position -> 3x1 position of center of sphere
%        radius -> radius of sphere
function drawSphere(position,radius)
    [X,Y,Z] = sphere;
    X = X * radius + position(1);
    Y = Y * radius + position(2);
    Z = Z * radius + position(3);
    surf(X,Y,Z);
end


function rob = createRobot()

    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);
%     L(5) = Link([0 0.4318 0 1.571]);
    
    rob = SerialLink(L, 'name', 'robot');

end
