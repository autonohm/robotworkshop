
% Funktion zur Erzeugung von Daten mit Pseudo-Rauschen
%
% Parameter:
% A: Zustandsübergangsmodell nxn-Matrix
% x: Zustand nx1-Vektor
% vals: Anzahl an Messwerten
% sigma: Standardabweichung des Rauschens
% data: nx2-Matrix mit verrauschten Messwerten
%
% Autor: Stefan May
% Datum: 15.02.2011

function data=generate_noisy_data(A, x, vals, sigma)
	for i=1:vals 
		x = A * x;
	        if(i==vals/2)
	        	x(4) = -1;
	        endif	
		rnd1 = rand;
		rnd2 = rand;
		data(i,1) = x(1) + sigma * sqrt(-2.0 * log(rnd1)) * sin(2.0 * pi * rnd2);

		rnd1 = rand;
		rnd2 = rand;
		data(i,2) = x(2) + sigma * sqrt(-2.0 * log(rnd1)) * sin(2.0 * pi * rnd2);
	end;
end;
