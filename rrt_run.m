function []=rrt_run

    function[] = rect(x,y,l,b)
      hold on
      rectangle('Position',[x,y,l,b],'FaceColor',[0 0.5 0.5])
    end

    function circle(x,y,r)
        ang=0:0.01:2*pi; 
        xp=r*cos(ang);
        yp=r*sin(ang);
        plot(x+xp,y+yp, '.r');
    end

figure;
axis([0,200,0,200])
set(gca, 'XTick', 0:10:200)
set(gca, 'YTick', 10:10:200)
grid ON
hold on
rect(130,70,20,60);
rect(70,135,60,20);

p_start = [30;160];
p_goal = [160;80];

rob.x = 30;
rob.y = 160;

param.obstacles =[130,70,20,60; 70,135,60,20;];
param.threshold = 2;
param.maxNodes = 800;
param.step_size = 5; 
param.neighbourhood = 5;
param.random_seed = 40;

plot(rob.x,rob.y,'.r')
text(rob.x-5, rob.y+9, 'Starting Point');
% plot(p_goal(1), p_goal(2), '.r')
circle(p_goal(1), p_goal(2), 2)
text(p_goal(1), p_goal(2), 'Goal');

result = PlanPathRRTstar(param, p_start, p_goal)   
end