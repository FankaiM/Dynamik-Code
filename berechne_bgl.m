function rob=berechne_bgl(rob)
%Berechnung der Bewegungsgleichungen des Systems (M, h und ddot_q)
%
% Alle notwendigen Funktionen werden von hier aus aufgerufen
%


%% 1. Berechnung des h-Vektors:
% Tipp: Nutzen Sie zur Berechnung von h die Funktion "berechne_id.m"
%rob.h=?;

%% 2. Berechnung der Massenmatrix
rob.M=zeros(rob.N_Q,rob.N_Q);

% Beitraege aller Koerper addieren
for i=1:length(rob.kl)
    %Anteil diese Koerpers
%   dM=?;

    %Anteil zur Gesamt-Massenmatrix addieren 
    rob.M=rob.M+dM;
end

%Die aktuellen Beschleunigungen berechnen
%Hier werden auch die Antriebsmomente der Regelung tau_reg beruecksichtigt

%rob.ddot_q=?;
end

 
