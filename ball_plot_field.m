function [ scratch ] = ball_plot_field(scratch, obj, tree, parents, obstacles, goal_state, goal_cost, goal_parent )
%PLOT_GRAPH Summary of this function goes here
%   Detailed explanation goes here

clf;
hold on;
for ii=2:size(tree,2) %root node has no parents -> start with 2
    src = tree(:,parents(ii));
    dst = tree(:,ii);
    draw_trajectory(obj, src, dst, 'blue', 1);
end


for ii=1:size(obstacles,1)
    obs = obstacles(ii,:);
    line([obs(1),        obs(1)+obs(3)], [obs(2),        obs(2)],         'Color','red');
    line([obs(1)+obs(3), obs(1)+obs(3)], [obs(2),        obs(2)+obs(4)],  'Color','red');
    line([obs(1)+obs(3), obs(1)],        [obs(2)+obs(4), obs(2)+obs(4)],  'Color','red');
    line([obs(1),        obs(1)],        [obs(2)+obs(4), obs(2)],         'Color','red');
end

if goal_cost < inf
    p = goal_parent;
    h = draw_trajectory(obj, tree(:,p), goal_state, 'green', 3);
    uistack(h, 'top')
    c = p;
    p = parents(c);
    while p > 0
        h = draw_trajectory(obj, tree(:,p), tree(:,c), 'green', 3);
        uistack(h, 'top')
        c = p;
        p = parents(c);
    end
end

hold off;
title(['cost: ', num2str(goal_cost)]);
drawnow

end

function [h] = draw_trajectory(obj,x0,x1,color, thickness)

    t = obj.evaluate_arrival_time(x0,x1);
    [states, ~] = obj.evaluate_states_and_inputs(x0,x1);
    X = [];
    Y = [];
    for jj=[0:1:t,t]
        p = states(jj);
        X = [X,p(1)];
        Y = [Y,p(2)];
    end
    h = line(X, Y, 'Color', color, 'LineWidth', thickness);

end
