function path = Astar(maze,start,ending)

startingNode = Node(start,NaN);
endingNode = Node(ending,NaN);

openList = {};
closedList = {};

openList{1} = startingNode;


while size(openList,2) > 0
    currentNode = openList{1};
    currentIndex = 1;
    % go through each open entry
    for index=1:size(openList)
        % get lowest f value
        if (openList{index}.f) < (currentNode.f)

            currentNode = openList{index};
            currentIndex = index;
        end
    end
    % remove currentNode from open list
    openList(currentIndex) = [];
    % and add it to closed list;
    closedList{end+1} = currentNode;
    
    
    if currentNode.position == endingNode.position
        path={};
        current=currentNode;
        while true
            try path{end+1} = current.position;
            catch
                break;
            end
            current = current.parent;
        end
        path = flip(path);
        return;
    end
        
    % Create a cell with children(like Trump):
    children = {};
    
    % Generate the adjacent squares:
    adjacentNodes = [0 1;1 1;1 0;1 -1;0 -1;-1 -1; -1 0; -1 1];
    for i=1:8
       newPosition = currentNode.position + adjacentNodes(i,:);
       % need a check if it's in the range:
       [row,col] = size(maze);
       if newPosition(1) > row || newPosition(1) < 1 ||...
               newPosition(2) > col || newPosition(2) < 1
           continue;
       end
       % check if walkable (i.e. unobstructed)
       if maze(newPosition(1),newPosition(2)) ~= 0
           continue;
       end
       

       % create a new node with currentNode as parent and with
       % determined position
       newNode = Node(newPosition,currentNode);
       % add this new node to children cell;
       children{i} = newNode;
    end

    % for each child
    for numChild=1:size(children,2)
        flag = 0;
        if isempty(children{numChild})
            continue;
        end
        % check each entry in Closed List
        for i=1:size(closedList,2)
            %if child is in closed list check next child
            if strcmpi(children{numChild},closedList{i}) 
                flag = 1;
                break;
            end
        end
       if flag == 1, continue; end
       % find distance from current node to child
       distance = pdist([currentNode.position;children{numChild}.position],'euclidean');
       % find distance from child to goal
       distanceToEnd = pdist([children{numChild}.position;endingNode.position],'euclidean');
       
       children{numChild}.g = currentNode.g + distance;
       children{numChild}.h = distanceToEnd;
       children{numChild}.f = children{numChild}.g + children{numChild}.h;
       % check for child open List
       for i=1:size(openList,2)
           % if the position of child already exists in open List
           if children{numChild}.position == openList{i}.position
               % if its g is higher, check next one
               if children{numChild}.g > openList{i}.g
                    flag = 1;
                    break;
               end
           end
       end
       if flag == 1, continue; end
       openList{end+1} = children{numChild}; 
    end

end
end



    