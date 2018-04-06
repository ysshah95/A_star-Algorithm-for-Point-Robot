function [StatusDR, NewNodeDR] = ActionMoveDownRight(CurrentNode)
x = CurrentNode(1);
y = CurrentNode(2);
if y == 0 || x == 250
    StatusDR = false;
    NewNodeDR = [];
else
    newX = x+1;
    newY = y-1;
    StatusDR = true;
    NewNodeDR = [newX, newY];
end
