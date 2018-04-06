function [StatusL, NewNodeL] = ActionMoveLeft(CurrentNode)
x = CurrentNode(1);
y = CurrentNode(2);
if x == 0
    StatusL = false;
    NewNodeL = [];
else
    newX = x-1;
    newY = y;
    StatusL = true;
    NewNodeL = [newX, newY];
end
     