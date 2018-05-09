%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Create Inverse Kinematic

%%% Link length of robotarm
l1 = 0.5;
l2 = 0.5;

%%% Target Point
x = 0.5;
y = 0.5;
z = 0.5;

%%% DH-Parameter and Assembly of robotarm
L(1) = Link([0 0 0 pi/2]);
L(2) = Link([0 0 l1 0]);
L(3) = Link([0 0 l2 -pi/2]);

robot = SerialLink(L, 'name', 'OHM');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Analytical Solution 
%%%% Start here









%%%% End here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Plot robot two time, because of deprecated teach command
q = [q1 q2 q3];

hFig = figure(1);
set(hFig, 'Position', [500 500 1000 1000])
robot.plot(q)

hFig = figure(2);
set(hFig, 'Position', [0 0 500 500])
robot.teach(q)
clear
