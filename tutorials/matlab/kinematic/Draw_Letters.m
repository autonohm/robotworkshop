%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Draw letters on a block
clear

%%% Load Manipulator and Update Configuration
mdl_puma560
p560.tool = SE3(0, 0, 0.2);

%%% Open plot in specific size 
hFig = figure(1);
set(hFig, 'Position', [500 500 1000 1000])

%%% Create Cube in Plot
P = [0.5,0,-0.5] ;           % center point of cube 
L = [0.5,1,0.3] ;            % cube dimensions 
O = P-L/2 ;                  % Get the origin of cube so that P is at center 
plotcube(L,O,.8,[1 0 0]);    % use function plotcube 
hold on


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Programm Movement of the robotarm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Calculate start position  
Tstart = transl(0.75, 0, 0) * trotx(0) * troty(pi/2) * trotz(0);
qstart = p560.ikine6s(Tstart);
p560.plot(qstart)


%%% Start here





%%% End here


clear
