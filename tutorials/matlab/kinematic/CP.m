function  CP( Tget1, Tget2, robot2 )

t1 = [0:0.05:1]';

%Create Continious Path Motion 
T = ctraj(Tget1, Tget2, length(t1));

%Create Trajectory
qback = robot2.ikine6s(T);


robot2.plot(qback)

end