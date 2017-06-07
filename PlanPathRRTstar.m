function total_cost = PlanPathRRTstar(param, p_start, p_goal)

% RRT* 
% credit : Anytime Motion Planning using the RRT*, S. Karaman, et. al.
% calculates the path using RRT* algorithm 
% but doesn't consider any obstacles here
% param : parameters for the problem 
%   1) threshold : stopping criteria (distance between goal and current
%   node)
%   2) maxNodes : maximum nodes for rrt tree (but not used it here)
%   3) neighborhood : distance limit used in finding neighbors
% p_start : [x;y] coordinates
% p_goal : [x;y] coordinates
% variable naming : when it comes to describe node, if the name is with
% 'node', it means the coordinates of that node or it is just an index of
% rrt tree
% rrt struct : 1) p : coordinate, 2) iPrev : parent index, 3) cost :
% distance
% for cost, Euclidean distance is considered.

    P =[];
    rrt = {};
    temp_node.p = p_start;
    temp_node.iPrev = 0;
    temp_node.cost = 0;
    rrt{1} = temp_node;    
    N = param.maxNodes; % iterations
    endcondition = param.threshold + 1;
    j = 1;
    
    while endcondition>param.threshold && j<=N     
        sample_node = getSample();
        plot(sample_node(1), sample_node(2), '.g');
        text(sample_node(1), sample_node(2), strcat('random',num2str(j)))
        nearest_node_ind = findNearest(rrt, sample_node);
        plot(rrt{nearest_node_ind}.p(1), rrt{nearest_node_ind}.p(2), '.g');
        text(rrt{nearest_node_ind}.p(1), rrt{nearest_node_ind}.p(2), strcat('nearest', num2str(j)));
        new_node = steering(rrt{nearest_node_ind}.p, sample_node);
        plot(new_node(1), new_node(2), '.g');
        text(new_node(1), new_node(2)+3, strcat('steered: new node', num2str(j)))
        
        % assume no obstacle
        neighbors_ind = getNeighbors(rrt, new_node)
        if(~isempty(neighbors_ind))
            parent_node_ind = chooseParent(rrt, neighbors_ind, nearest_node_ind,new_node);
            plot(rrt{parent_node_ind}.p(1), rrt{parent_node_ind}.p(2), '.g');
            text(rrt{parent_node_ind}.p(1), rrt{parent_node_ind}.p(2)+3, strcat('parent', num2str(j)));
        else
            parent_node_ind = nearest_node_ind;
        end
        rrt = insertNode(rrt, parent_node_ind, new_node);
        if (~isempty(neighbors_ind))
            rrt = reWire(rrt, neighbors_ind, parent_node_ind, length(rrt));
        end
        endcondition = norm(new_node-p_goal);
        j = j + 1;
    end
    setPath(rrt);
 
    
    function setPath(rrt)
        rob.x = rrt{1}.p(1); rob.y=rrt{1}.p(2);
        plot(rob.x,rob.y,'.b')
        for i = 2: length(rrt)
            p1 = rrt{i}.p;
            p2 = rrt{rrt{i}.iPrev}.p;
            drawnow
            rob.x = p1(1);
            rob.y = p1(2);
            plot(rob.x,rob.y,'.g');
            %plot([p1(1),p2(1)], [p1(2),p2(2)], 'b', 'LineWidth', 1);
            pause(0.1)
        end 
        
        i = length(rrt);
        total_cost = rrt{i}.cost
        while i ~= 0
            p11 = rrt{i}.p;
            i = rrt{i}.iPrev;
            if i ~= 0
                p22 = rrt{i}.p;
                %plot([p11(1),p22(1)], [p11(2),p22(2)], 'b', 'LineWidth', 3);
            end
            
        end
        
    end
    
    function new_node=steering(nearest_node, random_node)
       dist = norm(random_node-nearest_node);
       ratio_distance = param.step_size/dist;
       
       x = (1-ratio_distance).* nearest_node(1)+ratio_distance .* random_node(1);
       y = (1-ratio_distance).* nearest_node(2)+ratio_distance .* random_node(2);
       new_node = [x;y];
    end
    
    function rrt = reWire(rrt, neighbors, parent, new)
        for i=1:length(neighbors)
            cost = rrt{new}.cost + norm(rrt{neighbors(i)}.p - rrt{new}.p);
            if (cost<rrt{neighbors(i)}.cost)
                rrt{neighbors(i)}.iPrev = new;
            end
        end
    end
    

    function rrt = insertNode(rrt, parent, new_node)
        node.p = new_node;
        node.iPrev = parent;
        node.cost = rrt{parent}.cost + norm(rrt{parent}.p-new_node);
        rrt{end+1} = node;
    end
    
    function parent = chooseParent(rrt, neighbors, nearest, new_node)
        min_cost = getCostFromRoot(rrt, nearest, new_node);
        parent = nearest;
        for i=1:length(neighbors)
            cost = getCostFromRoot(rrt, neighbors(i), new_node);
            if (cost<min_cost)
               min_cost = cost;
               parent = neighbors(i);
            end
        end
    end
    
    function cost = getCostFromRoot(rrt, parent, child_node)       
       cost =  rrt{parent}.cost + norm(child_node - rrt{parent}.p);
    end

    function neighbors = getNeighbors(rrt, node)
        neighbors = [];
        for i = 1:length(rrt)
            rrt{i}.p
            dist = norm(rrt{i}.p-node);
            if (dist<=param.neighbourhood)
               neighbors = [neighbors i];
            end
        end        
    end
    
    function node = getSample()
        x = 0;
        y = 0;
        a = 0;
        b = 200;
        node = [x;y];
        node(1) = (b-a) * rand(1) + a;
        node(2) = (b-a) * rand(1) + a;  
    end
    
    
    function indx = findNearest(rrt, n)
        mindist = norm(rrt{1}.p - n);
        indx = 1;
        for i = 2:length(rrt)
            dist = norm(rrt{i}.p - n);
            if (dist<mindist)
               mindist = dist;
               indx = i;
            end
        end
    end 
    
end