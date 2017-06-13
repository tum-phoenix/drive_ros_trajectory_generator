function [X, Y, PHI] = getPointsFromState(r, delta)

P = zeros(numel(r), 3); %x, y, phi
phi = r(2);

%erster Punkt 
P(1, 1) = 0;
P(1, 2) = r(1);
P(1, 3) = phi;

%zweiter Punkt
P(2, 1) = P(1, 1) + delta*cos(r(2));
P(2, 2) = P(1, 2) + delta*sin(r(2));  
P(2, 3) = phi;
    
for s=3:numel(r)   
        dw = 2*acos(-delta*r(s)/2); % "-" wegen VZ-Definition der Kruemmung
        phi = phi + dw - pi; 
        P(s, 1) = P(s-1, 1) + delta*cos(phi);
        P(s, 2) = P(s-1, 2) + delta*sin(phi);
        P(s, 3) = phi;  
end  

X = P(:, 1);
Y = P(:, 2);
PHI = P(:, 3);