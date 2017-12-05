function rob = berechne_id(rob)
%Inverse Dynamik fuer Roboter rob berechnen
% Die Ergebnisse werden wiederum in der Struktur rob. gespeichert
%
% Im einzelnen werden Impuls- und Drallaenderung aller Koerper berechnet und
% ueber die Jacobi-Matrizen in die zwangsfreien Richtungen projiziert.
% Im Ergebnis wird die "linke Seite der Bewegungsgleichung"
% M*ddot_q + h berechnet und in tau_id gespeichert
% Es werden alle noetigen Groessen hier berechnet

%1. Mit Null initialisieren
rob.tau_id=zeros(rob.N_Q,1);

%2. Kinematik berechnen
rob=berechne_dk_positionen_vektorkette(rob);%%%veranderung
rob=berechne_dk_geschwindigkeiten(rob);
rob=berechne_dk_beschleunigungen(rob);
rob=berechne_dk_jacobis(rob,'ttt');%%%veranderung

%3. Berechnung fuer alle Koerper: Impuls- und Drallaenderung
for i=1:length(rob.kl)
    
    %Absolutbeschleunigung des Schwerpunkts:
    %%a_i_is=a_i_i+a_i_s
    %%a_i_is=a_i_i+omega_dot_i_i x r_is + omega_i_i x omega_i_i x r_is 
    Bi_ddot_r_is= tilde(rob.kl(i).Bi_dot_omega)*rob.kl(i).Bi_r_s+...
    tilde(rob.kl(i).Bi_dot_omega)*tilde(rob.kl(i).Bi_dot_omega)*rob.kl(i).Bi_r_s;
    rob.kl(i).Bi_ddot_r_s = rob.kl(i).Bi_ddot_r_i+Bi_ddot_r_is;
    
    %Impulsaenderung - Schwerkraft
    %%--Impulsaenderung
    pi=rob.kl(i).m*rob.kl(i).Bi_ddot_r_s;
    %%--eingepraegte Schwerkraft wird in i-KOSY transformiert
    Fe=rob.kl(i).A_i0*rob.kl(i).m*rob.B0_g;
   
    F = pi-Fe;
    
    %Drallaenderung - Moment der Schwerkraft
    %%--eingepraegte Moment bezuelich auf Schwerpunkt
    Me=tilde(rob.kl(i).Bi_r_s)*Fe;
    %%--Drallaenderung
    L_dot_s=rob.kl(i).I_o*rob.kl(i).Bi_dot_omega;
    
    T =L_dot_s-Me;
    
    %Projektion auf zwangsfreie Richtungen und Addition zu tau_id
    rob.tau_id= [rob.kl(i).Bi_Jt_o;rob.kl(i).Bi_Jr]'*[F;T];
end
end



