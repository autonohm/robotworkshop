function CP_pen( Tget1, Tget2, robot1 )


t1 = [0:0.05:1]';

%Create Continious Path Motion 
T = ctraj(Tget1, Tget2, length(t1));

%Create Trajectory
qback = robot1.ikine6s(T);

%Draw Trajectory 
for i=1:1:21 % depending if t = 41 or t1 = 21
    fkt=robot1.fkine(qback(i,:));
    cord(i,:) = fkt.t;  
    %Plot line
    plot2(cord, 'b', 'LineWidth', 2.5)
    robot1.plot(qback(i,:))
end


end

