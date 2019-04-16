% Find a path from qStart to qGoal using RRT.
% 
% input: rob -> a puma 560 robot
%        qStart -> 1x6 vector describing start configuration
%        qGoal -> 1x6 vector describing goal configuration
%        sphereCenters -> 3xN position of centers of all spherical obstacle
%        sphereRadii -> 1xN radius of all corresponding spherical obstacles
% output: qMilestones -> mx6 matrix of vertices along path from start to
%                        goal.
function qMilestones = rrt_algorithm(rob,qStart,qGoal,sphereCenters,sphereRadii)
    fprintf("Starting RRT\n");
    % Your code here
    nodes(1).parent=0;
    nodes(1).v=qStart;
    
    %fprintf("Generating configurations in free space from start to goal\n");
    index=1;
    % loop till qGoal is reached
    while ~isequal(nodes(index).v, qGoal)
        %generate random q. for every 10th iteration consider qGoal
        if mod(index,10) == 0
            fprintf("\tgenerated %d free configurations...\n",index);
            qRand = qGoal;
        else
            qRand = -pi + 2*pi*rand(6,1);
            qRand = qRand';
            if robotCollision(rob,qRand,sphereCenters,sphereRadii) == 1
                continue
            end
        end
        
        %find nearest node to qRand
        minDist=norm(qRand - nodes(1).v);
        nearestParent = 1;
        i=2;
        while i <= index
            actualDist=norm(qRand - nodes(i).v);
            if actualDist < minDist
                minDist=actualDist;
                nearestParent=i;
            end
            i = i+1;
        end
        
        %if edge to nearest node is in collision, then go to next loop
        if checkEdgeCollision(rob,qRand,nodes(nearestParent).v,sphereCenters,sphereRadii) == 1
            continue
        end
        
        %increment index and add qRand to nodes
        index = index+1;
        nodes(index).v=qRand;
        nodes(index).parent=nearestParent;
    end
    
    
    fprintf("Finding path from start to goal\n");
    %find path from start to goal
    nextNode = index;
    %if nodes(nextNode).v == qGoal
    if isequal(nodes(nextNode).v, qGoal)
        %add goal configuration
        qMilestones = nodes(nextNode).v;
        nextNode = nodes(nextNode).parent;
        while nodes(nextNode).parent ~= 0
            qMilestones = [nodes(nextNode).v ; qMilestones];
            nextNode = nodes(nextNode).parent;
        end
        %add start configuration
        qMilestones = [nodes(nextNode).v ; qMilestones];
    end
    
end
