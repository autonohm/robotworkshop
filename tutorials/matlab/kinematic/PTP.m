function  PTP( Tget1, Tget2, robot )

t1 = [0:0.05:1]';

qold = robot.ikine6s(Tget1);
qnew = robot.ikine6s(Tget2); 

qback = jtraj(qold, qnew, t1);

robot.plot(qback)

end

