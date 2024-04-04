% this is the main script that calls actions to the robot
% HW9
resetWorld
goHome
% stage A (three red cans)
disp("Stage A: Three Red Cans...")
pause(0.5)
pickAndDropHW9BILLY(25) 
pickAndDropHW9BILLY(24)
pickAndDropHW9BILLY(26)
% stage B (three yellow cans)
disp("Stage B: Three Yellow Cans... (third can unable to be performed)")
pause(0.5)
pickAndDropHW9BILLY(29)
pickAndDropHW9BILLY(27)
% stage C
disp("Stage C: Three Other Objects...")
pause(0.5)
pickAndDropHW9BILLY(20)
pickAndDropHW9BILLY(22)
pickAndDropHW9BILLY(33)