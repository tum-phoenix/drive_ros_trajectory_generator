clc;

laneModel = [0.1 0 1 -1 -0.4];
lanePieceLength = 0.2;
delta_x = 0;

r = [0, 0, 0, 0]';
Pk = eye(4);


Q = 0.01;
Rx = 0.02;
Ry = 0.03;

x_measure = 0.33;
y_measure = 0.1;



[r, Pk]  = objectTracker(1, laneModel, lanePieceLength, r, Pk, Q, Rx, Ry, x_measure, y_measure, delta_x);

for i=1:15
    clf;
    x_measure = x_measure;% + 0.01;
    [r, Pk]  = objectTracker(0, laneModel, lanePieceLength, r, Pk, Q, Rx, Ry, x_measure, y_measure, delta_x);
    [xp, yp, phi] = getPointsFromState(laneModel, lanePieceLength); 
    r
    plot(xp, yp, '-o');
    hold on;
    plot(x_measure, y_measure, '*')
    grid on;
    axis equal;
    
    waitforbuttonpress
end
r
Pk

[xp, yp, phi] = getPointsFromState(laneModel, lanePieceLength); 

plot(xp, yp, '-o');
hold on;
plot(x_measure, y_measure, '*')
grid on;
axis equal;