%% script di prova per testare il corretto funzionamento della funzione GetLosPoint
% define two points: 
P1 = eye(4);
P2 = eye(4);
P3 = eye(4);
P1(1:3, 4) = [0.6; -0.35; 0.5];
P2(1:3, 4) = [0.6; -0.35; 0.2];
P3(1:3, 4) = [0.6; 0.2; 0.];

LOS = GetLosPoint(P3, P1, P2);
display(LOS);