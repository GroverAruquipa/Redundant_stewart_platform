function stewie()

a = 10;

% Side lengths of the base frame
b = 15;
d = 1;

Xb = [
	(sqrt(3)/6)*(2*b + d)
	-(sqrt(3)/6)*(b - d)
    -(sqrt(3)/6)*(b + 2*d)
    -(sqrt(3)/6)*(b + 2*d)
    -(sqrt(3)/6)*(b - d)
    (sqrt(3)/6)*(2*b + d)
];
    
Yb = [
    d/2
    (b + d)/2
    b/2
    -b/2
    -(b + d)/2
    -d/2
];

Zb = [0 0 0 0 0 0];

% The pause time between each frame
pauseTime = 0.01;


% Move from the lowest pisition (a) to the highest position (b)
for k = 8:0.1:15
    L = [k,k,k,k,k,k];    
    stplot(L, a, b, d, Xb, Yb, Zb);
    title('Moving from the lowest position to the highest position');
    getframe;
    pause(pauseTime);
end

% Move to the most tilted position (c)
for k = 15:-0.1:8
    L = [k,k,k,k,15,15];
    stplot(L, a, b, d, Xb, Yb, Zb);
    title('Moving to the most tilted position');
	getframe;
    pause(pauseTime);
end

% Move back to the highest position (b)
for k = 8:0.1:15
    L = [k,k,k,k,15,15];
    stplot(L, a, b, d, Xb, Yb, Zb);
    title('Moving back to the highest position');
	getframe;
    pause(pauseTime);
end

% Move to the most twisted position (d)
for k = 15:-0.1:8
    L = [15,k,15,k,15,k];
    stplot(L, a, b, d, Xb, Yb, Zb);
    title('Moving to the most twisted position');
	getframe;
    pause(pauseTime);
end

% Move to the lowest position (a)
for k = 15:-0.1:8
    L = [k,8,k,8,k,8];    
    stplot(L, a, b, d, Xb, Yb, Zb);
    title('Moving to the lowest position');
	getframe;
    pause(pauseTime);
end

function stplot( L, a, b, d, Xb, Yb, Zb )
%STPLOT Plots a particular state of a Stewart platform. 
%   It takes the lengths of the prismatic joints (L) the platform
%   dimensions (a, b and d) and the base frame vertex coordinates 
%   (Xb, Yb and Zb) as parameters and plots the platform.

    % Solve the top plane
    [Xt, Yt, Zt] = stsolve(L, a, b, d);
    
    % Plot lower part
    fill3(Xb, Yb, Zb, [1,0.5,0.5]);
    hold on
    
    % Plot the lines
    stplotline(1, 1, Xb, Yb, Zb, Xt, Yt, Zt, [0.3,0.3,0.8]);
    stplotline(2, 1, Xb, Yb, Zb, Xt, Yt, Zt, [0.3,0.8,0.3]);
    stplotline(3, 2, Xb, Yb, Zb, Xt, Yt, Zt, [0.5,0.5,1]);
    stplotline(4, 2, Xb, Yb, Zb, Xt, Yt, Zt, [0.5,1,0.5]);
    stplotline(5, 3, Xb, Yb, Zb, Xt, Yt, Zt, [0.5,0.5,1]);
    stplotline(6, 3, Xb, Yb, Zb, Xt, Yt, Zt, [0.5,1,0.5]);
    
    % Plot upper part
    fill3(Xt, Yt, Zt, [0.5,1,1]);
    hold off
    grid on
    
    % Frames' alpha, axis' labels and size
    alpha(0.8);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis([-10 10 -10 10 0 12]);
    
function stplotline( bottomPoint, topPoint, Xb, Yb, Zb, Xt, Yt, Zt, color )
%STPLOTLINE Plots a line in space
%   Given the indices of a bottom and a top point, the base frame,
%   the upper frame and a color as parameters, stplotline plots
%   a prismatic joint for a Stewart platform.

    plot3([Xb(bottomPoint),Xt(topPoint)],...
        [Yb(bottomPoint),Yt(topPoint)],...
        [Zb(bottomPoint),Zt(topPoint)],...
        'LineWidth',5,'Color',color);
    
function [ Xt, Yt, Zt ] = stsolve( L, a, b, d )

    % Calculate heights and Xp, Yp vectors
    [h, Xp, Yp] = geval(L, b, d);

    % Create an anonymous function of stf only depending on Xt
    % to pass it to fsolve
    stewie = @(Xt) stf(Xt, a, h, Xp, Yp);
    
    % This guess generates imaginary residuals and is not to good
    % Xt = [sqrt(3)/6*a, -sqrt(3)/3*a, sqrt(3)/6*a ];
    
    % This is a better guess for fsolve
    Xt = Xp;
    
    Xt = fsolve(stewie,Xt);
    [Yt, Zt] = yz(Xt, h, Xp, Yp);
    
function [ h, Xp, Yp ] = geval( L, b, d )
%GEVAL Calculates h_i, X_P_i and Y_P_i for i=1..3
%   Given the lengths of the prismatic legs (vector L) and the dimensions
%   of the base frame (b, d), geval calculates the heights h(i) and points
%   (Xp(i), Yp(i)) for i=1,2,3

    zero = zeros(1,3);
    
    % Calculate the P vector
    P = zero;
    for i=1:3
        P(i) = (1/(2*b)) * (b^2 + L(2*i-1)^2 - L(2*i)^2 );
    end
    
    % Calculate the h vector
    h = zero;
    for i=1:3
        h(i) = sqrt(L(2*i-1)^2 - P(i)^2);
    end
    
    % Calculate the Xp vector
    Xp = zero;
    Xp(1) = (sqrt(3)/6) * (2*b + d - 3*P(1));
    Xp(2) = -(sqrt(3)/6) * (b + 2*d);
    Xp(3) = -(sqrt(3)/6) * (b - d - 3*P(3));
    
    % Calculate the Yp vector
    Yp = zero;
    Yp(1) = (1/2) * (d + P(1));
    Yp(2) = (1/2) * (b - 2*P(2));
    Yp(3) = -(1/2) * (b + d - P(3));

function LHS = stf(Xt, a, h, Xp, Yp)


    LHS = zeros(1,3);
    
    LHS(1) = a^2 + 2*Xt(1)*Xt(2) - 2*Xt(1)*(Xp(1) + sqrt(3)*(Yp(1)-Yp(2))) - 2*Xp(2)*Xt(2)...
        - ((sqrt(3)*Xp(1) - Yp(1) + Yp(2))^2 + (h(1)^2 + h(2)^2) - 4*Xp(1)^2 - Xp(2)^2)...
        + 2*sqrt( (h(1)^2 - 4*(Xt(1) - Xp(1))^2) * (h(2)^2 -(Xt(2)-Xp(2))^2) );
    
    LHS(2) = a^2 - 4*Xt(1)*Xt(3) - 2*Xt(1)*(Xp(1) - 3*Xp(3) + sqrt(3)*(Yp(1) - Yp(3)))...
        -2*Xt(3)*(-3*Xp(1)+Xp(3)+sqrt(3)*(Yp(1)-Yp(3)))...
        -((sqrt(3)*(Xp(1) + Xp(3)) - Yp(1) + Yp(3))^2 + (h(1)^2 + h(3)^2) - 4*Xp(1)^2 - 4*Xp(3)^2)...
        +2*sqrt( (h(1)^2 - 4*(Xt(1) - Xp(1))^2)*(h(3)^2 - 4*(Xt(3) - Xp(3))^2) );
    
    LHS(3) = a^2 + 2*Xt(2)*Xt(3) - 2*Xt(3)*(Xp(3) + sqrt(3)*(Yp(2) - Yp(3))) - 2*Xp(2)*Xt(2)...
        -((sqrt(3)*Xp(3) - Yp(2) + Yp(3))^2 + (h(2)^2 + h(3)^2) - Xp(2)^2 - 4*Xp(3)^2)...
        +2*sqrt( (h(2)^2 - (Xt(2) - Xp(2))^2) * (h(3)^2 - 4*(Xt(3) - Xp(3))^2));
    
function [ Yt, Zt ] = yz( Xt, h, Xp, Yp )
%YZ Summary of this function goes here
%   Detailed explanation goes here
    
    Yt(1) = sqrt(3)*Xt(1) - (sqrt(3)*Xp(1) - Yp(1));
    Yt(2) = Yp(2);
    Yt(3) = -sqrt(3)*Xt(3) + (sqrt(3)*Xp(3) + Yp(3));
    
    Zt(1) = sqrt( h(1)^2 - 4*(Xt(1) - Xp(1))^2 );
    Zt(2) = sqrt( h(2)^2 - (Xt(2) - Xp(2))^2 );
    Zt(3) = sqrt( h(3)^2 - 4*(Xt(3) - Xp(3))^2 );