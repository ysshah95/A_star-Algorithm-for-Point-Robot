function [StatusDL, NewNodeDL] = ActionMoveDownLeft(CurrentNode)
x = CurrentNode(1);
y = CurrentNode(2);
if x == 0 || y == 0
    StatusDL = false;
    NewNodeDL = [];
else
    newX = x-1;
    newY = y-1;
    StatusDL = true;
    NewNodeDL = [newX, newY];
end
