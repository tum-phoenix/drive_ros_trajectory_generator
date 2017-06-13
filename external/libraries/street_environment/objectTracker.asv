% init: [scalar] set to "1" once when object is first detected to initialize state vector, then set to "0"
% laneModel: [px1] state vector of the current lane model
% lanePieceLength: [scalar] length of linear pieces in the lane model
% r: [4x1] state vector of the object being tracked
%     [ s_x (arc length from vehicle to object in the diection of the lane model); 
%       v_x (velocity of object in direction of lane); 
%       s_y (lateral position of object relative to middle lane);
%       v_y (lateral velocity of object)]
% Pk:[4x4] covariance of the current state 
% Q: [scalar] standard deviation of the state transition (how reliable is delta_x?) 
% Rx: [scalar] standard deviation of the measurement in lane direction 
% Ry: [scalar] standard deviation of the measurement lateral to the lane 
% x_measure: [nx1] measured x values of the object being tracked
% y_measure: [nx1] measured y values of the object being tracked
% delta_x: [scalar] forward movement of the vehicle relative to the lane model

function [r, Pk] = objectTracker(init, laneModel, lanePieceLength, r, Pk, Q, Rx, Ry, x_measure, y_measure, delta_x, hasMeasurement)
% x und y Werte sind in Fzg-Koordinaten

%% find distance between lane model and object measurements to get the measurment of s (s_measure)
[xp, yp, phi] = getPointsFromState(laneModel, lanePieceLength); 
P = [xp, yp, phi];
D = 10000*[1 1 1 1];

for s=1:numel(laneModel)         
        dist_point = sqrt((P(s, 1)-(x_measure))^2 + (P(s, 2)-(y_measure))^2);
        if dist_point < D(3)
            D(1) = s;
            D(2) = 0;
            D(3) = dist_point;     
            D(4) = -sign(laneModel(min([numel(laneModel), s+1])));  % positive Krümmung -> vorzeichen von d negativ (und anders rum)     
        end            
        if s > 1
            [dist_line, lambda, sgn] = d_line_point([P(s-1, 1), P(s-1, 2)], [P(s, 1), P(s, 2)], [x_measure, y_measure]);
            if dist_line < D(3) && lambda > 0 && lambda < 1
                D(1) = s-1;
                D(2) = lambda;
                D(3) = dist_line; 
                D(4) = sgn;  % vorzeichen von d
            end
        end     
end

y_measure = D(4)*D(3);
x_measure = (D(1) + D(2)-1)*lanePieceLength;

% initialize the state vector on first object detection
if init == 1
    r = [x_measure; 0; y_measure; 0];
    return
end


%% state space model matrices
T = 0.01;
A =    [1, T, 0, 0;
        0, 1, 0, 0;
        0, 0, 1, T;
        0, 0, 0, 1];    
B = [-1; 0; 0; 0];    
H = [1, 0, 0, 0;
     0, 0, 1, 0];

% covariance of the state transition
d = Q^2;
QQ = [  d,     d*T,   0,     0;
        d*T,   d*T^2, 0,     0;
        0,     0,     d,     d*T;
        0,     0,     d*T,   d*T^2];

%% Kalman filter 

% prediction
r = A*r + B*delta_x;
Pk = A*Pk*A' + QQ;

% kalman gain
S = H*Pk*H' + [Rx^2, 0; 0 Ry^2];
K = Pk*H'/S;

% update
if hasMeasurement
    r = r + K*([x_measure; y_measure] - H*r);    
end

Pk = Pk - K*S*K';

