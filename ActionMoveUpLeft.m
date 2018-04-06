function [StatusUL, NewNodeUL] = ActionMoveUpLeft(CurrentNode)
x = CurrentNode(1);
y = CurrentNode(2);
if y == 150 || x == 0
    StatusUL = false;
    NewNodeUL = [];
else
    newX = x-1;
    newY = y+1;
    StatusUL = true;
    NewNodeUL = [newX, newY];
end
