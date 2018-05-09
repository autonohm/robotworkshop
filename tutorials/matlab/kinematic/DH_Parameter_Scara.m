%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Build a SCARA robotarm with DH parameter


%%% Write your solution between the two brackets
links = [


    ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Assembly of robot
scara = SerialLink(links, 'name', 'Scara' );

%%%Joint Angles - starting angles
 q = [0 0.1 0 0]; %Scara

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Plot robot two time, because of deprecated teach command
hFig = figure(1);
set(hFig, 'Position', [500 500 1000 1000])
scara.plot(q)

hFig = figure(2);
set(hFig, 'Position', [0 0 500 500])
scara.teach(q)

clear

