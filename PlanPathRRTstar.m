function result = PlanPathRRTstar(param, p_start, p_goal)

% RRT* 
% credit : Anytime Motion Planning using the RRT*, S. Karaman, et. al.
% calculates the path using RRT* algorithm 
% param : parameters for the problem 
%   1) threshold : stopping criteria (distance between goal and current
%   node)
%   2) maxNodes : maximum nodes for rrt tree 
%   3) neighborhood : distance limit used in finding neighbors
%   4) obstacle : must be rectangle-shaped #limitation
%   5) step_size : the maximum distance that a robot can move at a time
%   (must be equal to neighborhood size) #limitation
%   6) random_seed : to control the random number generation
% p_start : [x;y] coordinates
% p_goal : [x;y] coordinates
% variable naming : when it comes to describe node, if the name is with
% 'node', it means the coordinates of that node or it is just an index of
% rrt tree
% rrt struct : 1) p : coordinate, 2) iPrev : parent index, 3) cost :
% distance
% obstacle can only be detected at the end points but not along the line
% between the points
% for cost, Euclidean distance is considered.
% output : cost, rrt, time_taken 
% whether goal is reached or not, depends on the minimum distance between
% any node and goal 

field1 = 'p';
field2 = 'iPrev';
field3 = 'cost';
field4 = 'goalReached';

rng(param.random_seed);
tic;
start();

    function start()      
        rrt(1) = struct(field1, p_start, field2, 0, field3, 0, field4, 0);  
        N = param.maxNodes; % iterations
        j = 1;

%         while endcondition>param.threshold %&& j<=N     
        while j<=N   
            sample_node = getSample();
%             plot(sample_node(1), sample_node(2), '.g');
%             text(sample_node(1), sample_node(2), strcat('random',num2str(j)))
            nearest_node_ind = findNearest(rrt, sample_node);
%             plot(rrt(nearest_node_ind).p(1), rrt(nearest_node_ind).p(2), '.g');
%             text(rrt(nearest_node_ind).p(1), rrt(nearest_node_ind).p(2), strcat('nearest', num2str(j)));
            new_node = steering(rrt(nearest_node_ind).p, sample_node);
            if (isObstacleFree(new_node)==1)
%                 plot(new_node(1), new_node(2), '.g');
%                 text(new_node(1), new_node(2)+3, strcat('steered: new node', num2str(j)))
                neighbors_ind = getNeighbors(rrt, new_node);
                if(~isempty(neighbors_ind))
                    parent_node_ind = chooseParent(rrt, neighbors_ind, nearest_node_ind,new_node);
%                     plot(rrt(parent_node_ind).p(1), rrt(parent_node_ind).p(2), '.g');
%                     text(rrt(parent_node_ind).p(1), rrt(parent_node_ind).p(2)+3, strcat('parent', num2str(j)));
                else
                    parent_node_ind = nearest_node_ind;
                end
                rrt = insertNode(rrt, parent_node_ind, new_node);
                if (~isempty(neighbors_ind))
                    rrt = reWire(rrt, neighbors_ind, parent_node_ind, length(rrt));
                end
                if norm(new_node-p_goal) == param.threshold
                    rrt = setReachGoal(rrt);
                end
            end
            j = j + 1;
        end
        setPath(rrt);
%         text1 = strcat('Total number of generated nodes:', num2str(j-1))
%         text1 = strcat('Total number of nodes in tree:', length(rrt))
    end
    
    function rrt=setReachGoal(rrt)
        rrt(end).goalReached = 1;
    end
    
    
    function setPath(rrt)       
        for i = 1: length(rrt)-1
            p1 = rrt(i).p;
            rob.x = p1(1); rob.y=p1(2);
            plot(rob.x,rob.y,'.b')
            child_ind = find([rrt.iPrev]==i);
            for j = 1: length(child_ind)
                p2 = rrt(child_ind(j)).p;
                plot([p1(1),p2(1)], [p1(2),p2(2)], 'b', 'LineWidth', 1);
            end
        end 
        
        [cost,i] = getFinalResult(rrt);
        result.cost = cost;
        result.rrt = [rrt.p];
        while i ~= 0
            p11 = rrt(i).p;
            plot(p11(1),p11(2),'b', 'Marker','.', 'MarkerSize', 30);
            i = rrt(i).iPrev;
            if i ~= 0
                p22 = rrt(i).p;                
                plot(p22(1),p22(2),'b', 'Marker', '.', 'MarkerSize', 30);
%                 plot([p11(1),p22(1)],[p11(2),p22(2)], 'b', 'LineWidth', 3);
            end 
        end  
        result.time_taken = toc;
        
        
    end

    function [value,min_node_ind] = getFinalResult(rrt)
        goal_ind = find([rrt.goalReached]==1);
        if ~(isempty(goal_ind))
            disp('Goal has been reached!');
            rrt_goal = rrt(goal_ind);
            value = min([rrt_goal.cost]);
            min_node_ind = find([rrt.cost]==value);
            if length(min_node_ind)>1
                min_node_ind = min_node_ind(1);
            end
        else
            disp('Goal has not been reached!');
            for i =1:length(rrt)
                norm_rrt(i) = norm(p_goal-rrt(i).p);
            end
            [value,min_node_ind]= min(norm_rrt); 
            value = rrt(min_node_ind).cost;
        end
    end
    
    % if it is obstacle-free, return 1.
    % otherwise, return 0
    function free=isObstacleFree(node_free)
        free = 1;
        for i = 1: length(param.obstacles(:,1))
            obstacle = param.obstacles(i,:);
            op1 = [obstacle(1), obstacle(2)];
            op2 = [op1(1)+obstacle(3), op1(2)];
            op3 = [op2(1), op1(2) + obstacle(4)];
            op4 = [op1(1), op3(2)];
            
            nx = node_free(1);
            ny = node_free(2);
            
            if ((nx>=op1(1) && nx<=op2(1)) && (ny>=op1(2) && ny<=op4(2)))
                free = 0;
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
            cost = rrt(new).cost + norm(rrt(neighbors(i)).p - rrt(new).p);
            
            if (cost<rrt(neighbors(i)).cost)
%                 if norm(rrt(new).p-rrt(neighbors(i)).p)<param.step_size
% %                     plot(rrt(neighbors(i)).p(1), rrt(neighbors(i)).p(2), '.b');
%                     rrt(neighbors(i)).p = steering(rrt(new).p, rrt(neighbors(i)).p);
%                 end
%                 plot(rrt(neighbors(i)).p(1), rrt(neighbors(i)).p(2), '.m');
                rrt(neighbors(i)).iPrev = new;
                rrt(neighbors(i)).cost = cost;
            end
        end
    end
    

    function rrt = insertNode(rrt, parent, new_node)
        rrt(end+1) = struct(field1, new_node, field2, parent, field3, rrt(parent).cost + norm(rrt(parent).p-new_node), field4, 0);
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
       cost =  rrt(parent).cost + norm(child_node - rrt(parent).p);
    end

    function neighbors = getNeighbors(rrt, node)
        neighbors = [];
        for i = 1:length(rrt)
            dist = norm(rrt(i).p-node);
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
        mindist = norm(rrt(1).p - n);
        indx = 1;
        for i = 2:length(rrt)
            dist = norm(rrt(i).p - n);
            if (dist<mindist)
               mindist = dist;
               indx = i;
            end
        end
    end 
    
end