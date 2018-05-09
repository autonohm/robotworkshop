%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Build Puma560 robotarm with DH parameter

%%% Write your solution between the two brackets
links = [

     ];
  
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Assembly of robot
robot = SerialLink(links, 'name', 'Puma 560');

%%%Joint Angles - starting angles
q = [0 0 0 0 0 0]; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Plot robot two time, because of deprecated teach command
hFig = figure(1);
set(hFig, 'Position', [500 500 1000 1000])
robot.plot(q, 'jvec', 'arrow')

hFig = figure(2);
set(hFig, 'Position', [0 0 500 500])
robot.teach(q)
clear

