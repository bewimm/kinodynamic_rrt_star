function [ ] = plot_graph( tree, parents )
%PLOT_GRAPH Summary of this function goes here
%   Detailed explanation goes here

clf;
hold on;
for ii=2:size(tree,2) %root node has no parents -> start with 2
    a = tree(1:3,ii);
    b = tree(1:3,parents(ii));
    line([a(1),b(1)], [a(2),b(2)], [a(3),b(3)]);
end
hold off;
drawnow

end

