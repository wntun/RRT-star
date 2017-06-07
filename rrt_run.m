function []=rrt_run

figure;
axis([0,200,0,200])
set(gca, 'XTick', 0:10:200)
set(gca, 'YTick', 10:10:200)
grid ON
hold on


p_start = [30;160];
p_goal = [160;30];

rob.x = 30;
rob.y = 160;

plot(rob.x,rob.y,'.r')
text(rob.x-5, rob.y+9, 'Starting Point');
plot(p_goal(1), p_goal(2), '.r')
text(p_goal(1), p_goal(2), 'Goal');


param.threshold = 7;
param.maxNodes = 2;
param.step_size = 15; 
param.neighbourhood = 20;

cost = PlanPathRRTstar(param, p_start, p_goal)

   
end