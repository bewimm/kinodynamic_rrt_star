function [ scratch ] = quad_plot_field(scratch, obj, tree, parents, obstacles, waypoints, goal_state, goal_cost, goal_parent )
%PLOT_GRAPH Summary of this function goes here
%   Detailed explanation goes here

if ~isa(scratch,'struct') 
    %plot obstacles and waypoints    
    hold on;
    scratch = struct('figure_handle',gcf(),'last_parents', -1, 'trajectory_handles', -1, 'last_cost', inf, 'path_handles', []);
    for ii=1:size(obstacles,1)
        obs = obstacles(ii,:);   
        obs(1:3) = obs(1:3)-obs(4:6)/2;
        
        a = obs(1:3);
        b = obs(1:3)+[obs(4),0,0];
        c = obs(1:3)+[obs(4),obs(5),0];
        d = obs(1:3)+[0,obs(5),0];
        e = obs(1:3)+[0,0,obs(6)];
        f = e+[obs(4),0,0];
        g = e+[obs(4),obs(5),0];
        h = e+[0,obs(5),0];
        
        plot_quad(a,b,c,d,'red'); %bottom
        plot_quad(e,f,g,h,'red'); %top
        plot_quad(a,b,f,e,'red'); %front
        plot_quad(d,c,g,h,'red'); %back
        plot_quad(a,d,h,e,'red'); %left
        plot_quad(b,c,g,f,'red'); %right
        
    end
    
    waypoint_radius = 0.3;
    for ii=1:size(waypoints,1)
        [x,y,z] = sphere(10);
        mesh(x*waypoint_radius+repmat(waypoints(ii,1),size(x)),y*waypoint_radius+repmat(waypoints(ii,2),size(y)),z*waypoint_radius+repmat(waypoints(ii,3),size(z)))
    end     
end

%clf;
%hold on;
scratch.trajectory_handles = [scratch.trajectory_handles, -1];
scratch.last_parents = [scratch.last_parents, -1];
if false
    idx = find(scratch.last_parents~=parents);
    for ii=1:length(idx)
        
        changed_idx = idx(ii);
        src = tree(:,parents(changed_idx));
        dst = tree(:,changed_idx);
        draw_trajectory(obj, src, dst, 'blue', 1, scratch.trajectory_handles(changed_idx));
    end
    scratch.last_parents = parents;
end

if goal_cost < scratch.last_cost
    
    for ii=1:length(scratch.path_handles)
       delete(scratch.path_handles(ii)); 
    end
    scratch.path_handles = [];
    
    p = goal_parent;
    h = draw_trajectory(obj, tree(:,p), goal_state, 'green', 3);
    scratch.path_handles = [scratch.path_handles,h];
    uistack(h, 'top')
    c = p;
    p = parents(c);
    while p > 0
        h = draw_trajectory(obj, tree(:,p), tree(:,c), 'green', 3);
        scratch.path_handles = [scratch.path_handles,h];
        uistack(h, 'top')
        c = p;
        p = parents(c);
    end
    scratch.last_cost = goal_cost;
end

set(0, 'CurrentFigure', scratch.figure_handle);
xlabel('x');
ylabel('y');
zlabel('z');
title(['cost: ', num2str(goal_cost)]);
daspect([1 1 1]);
%hold off;
drawnow

end

function [] = plot_quad(a,b,c,d, color)    
    p = [a;b;c;d]; 
    fill3(p(:,1),p(:,2),p(:,3), zeros(4,1), 'FaceColor', [.8,.8,.8], 'EdgeColor', color);
end

function [h] = draw_trajectory(obj,x0,x1,color, thickness, old_handle)

    t = obj.evaluate_arrival_time(x0,x1);
    [states, ~] = obj.evaluate_states_and_inputs(x0,x1);
    X = [];
    Y = [];
    Z = [];
    for jj=[0:t/10:t,t]
        p = states(jj);
        X = [X,p(1)];
        Y = [Y,p(2)];
        Z = [Z,p(3)];
    end
    
    if exist('old_handle','var') && old_handle ~= -1
        set(old_handle, 'XData', X);
        set(old_handle, 'YData', Y);
        set(old_handle, 'ZData', Z);
        h = old_handle;
    else
        h = line(X, Y, Z, 'Color', color, 'LineWidth', thickness);
    end
    
end