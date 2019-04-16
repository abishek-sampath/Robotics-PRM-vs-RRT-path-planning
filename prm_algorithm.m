% Find a path from qStart to qGoal using PRM.
%
% input: rob -> a puma 560 robot
%        qStart -> 1x6 vector describing start configuration
%        qGoal -> 1x6 vector describing goal configuration
%        sphereCenters -> 3xN position of centers of all spherical obstacle
%        sphereRadii -> 1xN radius of all corresponding spherical obstacles
% output: qMilestones -> mx6 matrix of vertices along path from start to
%                        goal.
function qMilestones = prm_algorithm(rob,qStart,qGoal,sphereCenters,sphereRadii)
    fprintf("Starting PRM\n");
    % add the start configuration to free configurations
    cFree = qStart;
    % max number of free configurations
    cFreeMaxCount = 100;
    % initialize the adjacency matrix for Graph
    adjMatrix = zeros(cFreeMaxCount);
    fprintf("Running algorithm to generate %d free configurations\n", cFreeMaxCount);
    % max distance between neighbouring configurations
    maxNearDist = 6.0;

    n = 1;
    while n < cFreeMaxCount
        if n < cFreeMaxCount - 1
            % generate a random configuration
            qRand = -pi + 2*pi*rand(6,1);
            qRand = qRand';
        else
            % add the goal configuration
            qRand = qGoal;
        end

        % check if the configuration is already in the roadmap
        if ismember(qRand, cFree, 'rows') == 0
            qRandCoord = transl(rob.fkine(qRand));   %check this

            % check if the configuration is not on a obstacle
            if robotCollision(rob,qRand,sphereCenters,sphereRadii) == 0
                [cFreeCount, ~] = size(cFree);
                %get nearby configurations index
                qNear = [];
                for i = 1 : cFreeCount
                    qNearbyConfig = cFree(i, :);
                    nearDist = norm(qRand - qNearbyConfig);
                    qNearbyConfigCoord = transl(rob.fkine(qNearbyConfig));
                    %nearDist = norm(qRandCoord - qNearbyConfigCoord);

                    % find valid nearby configurations
                    if nearDist < maxNearDist
                        if checkEdgeCollision(rob,qRand,qNearbyConfig,sphereCenters,sphereRadii) == 0
                            qNear = [qNear i];
                        end
                    end
                end
                
                %add configuration
                cFree = [cFree; qRand];
                n = n + 1;
                if mod(n,20) == 0
                    fprintf("\tgenerated %d free configurations...\n",n);
                end

                % update the adjacency matrix
                if ~isempty(qNear)
                    for j = 1 : length(qNear)
                        adjMatrix(n, qNear(j)) = 1;
                        adjMatrix(qNear(j), n) = 1;
                    end
                end
            end
        end
    end
    
    fprintf("Generated adjacency matrix for graph. Creating Graph\n");
    % create a graph from the adjacency matrix
    G = graph(adjMatrix);
    % find the shortest path using djikistra's algorithm
    fprintf("Finding shortest path using Dijkstra's method\n");
    path = shortestpath(G, 1, cFreeMaxCount, 'Method', 'positive');
    % return the configurations in the path
    qMilestones = cFree(path, :);

end

