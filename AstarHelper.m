function COORDINATES = AstarHelper(start_point, end_point, wallFence)

% A* algorithm
%  empty arrays
InitialNode = zeros(size(wallFence,1)+2,3);

InitialNode(1,1:2) = [start_point(1), start_point(2)];
InitialNode(2:size(wallFence,1)+1,1:2) = wallFence;
InitialNode(size(wallFence,1)+2,1:2) = [end_point(1), end_point(2)];

%Infinite value
InitialNode(:,3) = Inf*ones(size(InitialNode,1),1);

% checking visibility
visible_nodes_ID = zeros(1,size(InitialNode,1));
NeighbourLibrary =  zeros(size(InitialNode,1),size(InitialNode,2),size(InitialNode,1));
visible_index = 0;

for RefNodeId = 1:size(InitialNode,1)
    combined_nodes = InitialNode;    
    for target_ID = 1:size(InitialNode,1)
        observer_state = InitialNode(RefNodeId,:);
        current_target_node = InitialNode(target_ID,:);
        visibility = visible(observer_state, current_target_node, wallFence);

        if visibility == 1
            visible_index = visible_index + 1;
            visible_nodes_ID(visible_index) = target_ID;
            combined_nodes(target_ID,3) = sqrt((current_target_node(1) - observer_state(1))^2 + (current_target_node(2) - observer_state(2))^2);                     
        end       
    end   
    NeighbourLibrary(:,:,RefNodeId) = combined_nodes;    
end
unvisited_nodes = zeros(1,size(InitialNode,1));
for index = 1:size(InitialNode)
    unvisited_nodes(index) = index;
end
shortestPath = NeighbourLibrary(:,:,1);
shortestPath(:,4) = 1;
current_node = 1;
unvisited_nodes = setdiff(unvisited_nodes, current_node);
while size(unvisited_nodes,2) > 0
    cumulative_distances = shortestPath(unvisited_nodes,3);
    [~, current_node_ID_index] = min(cumulative_distances);
    current_node_ID = unvisited_nodes(current_node_ID_index);
    unvisited_nodes = setdiff(unvisited_nodes, current_node_ID); 
    for IndexUnvisitedNode = 1:size(unvisited_nodes,2)
        target_node_ID = unvisited_nodes(IndexUnvisitedNode);
        if NeighbourLibrary(target_node_ID,3,current_node_ID) < Inf
            cumulative_distance_to_current_node = shortestPath(current_node_ID,3);
            distance_from_current_to_target_node = NeighbourLibrary(target_node_ID,3,current_node_ID);
            new_cumulative_distance = cumulative_distance_to_current_node + distance_from_current_to_target_node;
            previous_cumulative_distance_to_target_node = shortestPath(target_node_ID,3);
            if new_cumulative_distance < previous_cumulative_distance_to_target_node
                shortestPath(target_node_ID,3) = new_cumulative_distance;    
 
                shortestPath(target_node_ID,4) = current_node_ID;     
            end           
        end    
    end   
end
path = zeros(1,size(shortestPath,1));
path_index = 1;
path(path_index) = size(InitialNode,1);
while path(path_index) > 1   
    path_index = path_index + 1;
    path(path_index) = shortestPath(path(path_index-1),4);
    
end
path = fliplr(path(1:path_index));
COORDINATES(:,1) = InitialNode(path,1);
COORDINATES(:,2) = InitialNode(path,2);
end 