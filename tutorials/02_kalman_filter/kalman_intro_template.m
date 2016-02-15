clear all;

% Anzahl Messwerte
vals    = 100;

% Streuung der Messwerte
sigma = 5.0;

% Zustandsübergangsmodel
dT = 1.0;
A = [1    0    dT   0;
	0    1    0     dT;
	0    0    1     0;
	0    0    0     1];

% Realer Zustand
x = [0 0 1 1]';

% Messwerte generieren
data = generate_noisy_data(A, x, vals, sigma);
plot(data(:,1),data(:,2), 'x');

% Schätzung des Anfangszustands
x = [0 0 0 0]';

% Nur die Position ist messbar, nicht die Geschwindigkeit
H = [1 0 0 0; 0 1 0 0];

% Messrauschen
R = [sigma 0; 0 sigma];

% Systemrauschen
q = 0.0001; 
Q = [0 0 0 0; 0 0 0 0; 0 0 q 0; 0 0 0 q];

% Kovarianz
P = eye(4);

% Implementieren Sie hier den Kalmanfilter
for i=1:vals 
  x_prio = A * x; 
  y = H * x_prio;    
  P = A * P * A' + Q;  
  K = P * H' * inv(H*P*H' +R);
	x = x_prio + K * (data(i,:)' - y);
	P = (eye(4) - K * H) * P;
  data_kalman(i,:) = x; 
end; 



hold on;
plot(data_kalman(:,1),data_kalman(:,2), 'r');
hold off;