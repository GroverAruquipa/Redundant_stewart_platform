
%% Motion Trajectory
points = 200;
t = linspace(0,8,points);
x = 0.04*sin(t);
%x = zeros(1,length(t));     % surge [m]
y = zeros(1,length(t));     % sway [m]
z = zeros(1,length(t));     % heave [m]
%z = 0.015*sin(t);

pitch = 0.33*sin(t);  % pitch [rad]
roll = -0.35*sin(t);    % roll [rad]
%pitch = zeros(1,length(t));
%roll = zeros(1,length(t));
yaw = zeros(1,length(t));    % yaw [rad]
%yaw = 0.9*sin(t);
%% Build Parameters
% Angular coords of base and platform attachment points [deg]
% beta_b = [77, 103, 197, 223, 317, 343];
% beta_p = [37.5, 142.5, 157.5, 262.5, 277.5, 22.5];
beta_b = [0, 60, 120, 180, 240, 300];
beta_p = [15, 45, 135, 165, 255, 285];
beta = [-120 180 0 -60 120 60];     % Servo arm plane angles, lies in x-y plane

% Original Build
% Rb = 0.07; % Base radius [m]
% Rp = 0.04; % Platform radius [m]
% a = 0.025;
% s = 0.140;
% a = 0.0125; % 12.5 mm (short) or 14 mm (long) as measured on Servo 94102


Rb = 0.070; % Base radius [m] %0.07
Rp = 0.040; % Platform radius [m] %0.04
a = 0.03; % servo operating arm [m] %0.03
s = 0.120; % servo operating leg [m] %0.25

% ith position of servo arm on base
b = [Rb*cosd(beta_b(1)), Rb*sind(beta_b(1)), 0;
     Rb*cosd(beta_b(2)), Rb*sind(beta_b(2)), 0;
     Rb*cosd(beta_b(3)), Rb*sind(beta_b(3)), 0;
     Rb*cosd(beta_b(4)), Rb*sind(beta_b(4)), 0;
     Rb*cosd(beta_b(5)), Rb*sind(beta_b(5)), 0;
     Rb*cosd(beta_b(6)), Rb*sind(beta_b(6)), 0];   

b = b';

% ith position of anchor point on platform
p = [Rp*cosd(beta_p(1)), Rp*sind(beta_p(1)), 0;
     Rp*cosd(beta_p(2)), Rp*sind(beta_p(2)), 0;
     Rp*cosd(beta_p(3)), Rp*sind(beta_p(3)), 0;
     Rp*cosd(beta_p(4)), Rp*sind(beta_p(4)), 0;
     Rp*cosd(beta_p(5)), Rp*sind(beta_p(5)), 0;
     Rp*cosd(beta_p(6)), Rp*sind(beta_p(6)), 0];

p = p';

xp = p(1,:);
yp = p(2,:);
zp = p(3,:);

xb = b(1,:);
yb = b(2,:);
zb = b(3,:);


h0 = 0.91*sqrt(a^2 + s^2);

alfa = zeros(length(z),6);
hFig = figure(1);
set(hFig, 'Position', [0 0 1400 800])
vectx=[0];
vecty=[0];
vectz=[0];

    writerObj = VideoWriter('hexapo-sim2.avi');
    writerObj.FrameRate = 10;
    open(writerObj);


for k = 1:length(z);
    

    theta = pitch(k);
    phi = roll(k);
    psi = yaw(k);
    
    % Base to platform rotation matrix
    Rpb = [cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
           cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
           -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)        ];

    %% Effective leg lengths
    T = [x(k) y(k) h0(1)+z(k)]';
    q = repmat(T,1,6) + Rpb*p;  % Platform coords in base framework
    xq = q(1,:);
    yq = q(2,:);
    zq = q(3,:);
    
    l = q - b;  % leg lengths
    %% Servo angles
    L = sum(l.*l) - (s^2 - a^2);
    M = 2*a*(zq - zb);
    N = 2*a*(cosd(beta).*(xq - xb) + sind(beta).*(yq - yb));
    alpha = asind(L./sqrt(M.^2 + N.^2)) - atand(N./M);      % servo angles
    alfa(k,:) = alpha;
    
    % Servo arm coords
    xa = a*cosd(alpha).*cosd(beta) + xb;
    ya = a*cosd(alpha).*sind(beta) + yb;
    za = a*sind(alpha) + zb;
    
    %% Plot
    clf
    
    % Base
    plot3(b(1,:),b(2,:),b(3,:),'k','LineWidth',1.5);
    grid on; hold on; axis equal;
    plot3([b(1,1),b(1,6)],[b(2,1),b(2,6)],[b(3,1),b(3,6)],'k','LineWidth',1.5);
    
    % Platform
    plot3(q(1,:),q(2,:),q(3,:),'r','LineWidth',3);
    %plot3([q(1,1),q(1,6)],[q(2,1),q(2,6)],[q(3,1),q(3,6)],'r','LineWidth',3);
    
    for j = 1:6
        % Legs
        plot3([b(1,j),q(1,j)],[b(2,j),q(2,j)],[b(3,j),q(3,j)],'b','LineWidth',1.5);
        
        % Servo arms & legs
       % plot3([b(1,j),xa(j)],[b(2,j),ya(j)],[b(3,j),za(j)],'b','LineWidth',3);
        %plot3([q(1,j),xa(j)],[q(2,j),ya(j)],[q(3,j),za(j)],'m','LineWidth',3);
    end
    
    if sum(abs(alpha) > 50)
       % title('WARNING: Servo angles exceeded');
    end
    xmin = -Rb*1.05;
    xmax = Rb*1.05;
    ymin = xmin;
    ymax = xmax;
    zmin = -0.03;
    zmax = h0*1.25;
    axis([xmin xmax ymin ymax zmin zmax])
    
    top = [0,90];
    side = [0,0];
    iso = [45,45];
    
    front = [90,0];
    top2 = [90,90];
    iso = [45,45];
    vid = [145,15]; % video view
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%%% Vectors for center of mobile platform %%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x_center=(q(1,1)+q(1,2)+q(1,3)+q(1,4)+q(1,5)+q(1,6))/6;
    y_center=(q(2,1)+q(2,2)+q(2,3)+q(2,4)+q(2,5)+q(2,6))/6;
    z_center=(q(3,1)+q(3,2)+q(3,3)+q(3,4)+q(3,5)+q(3,6))/6;
    A7=[x_center y_center z_center]';
    B7=[0 0 0]';
    plot3([x_center 0],[y_center 0],[z_center 0],'r','LineWidth',3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%In order to calculate the inverse jacobian matrix%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    A1=q(:,1); A2=q(:,2); A3=q(:,3); A4=q(:,4); A5=q(:,5); A6=q(:,6);
    B1=b(:,1); B2=b(:,2); B3=b(:,3); B4=b(:,4); B5=b(:,5); B6=b(:,6);
    ijb=inverse_jacob_hexapod2(A1,A2,A3,A4,A5,A6,A7,B1,B2,B3,B4,B5,B6,B7);
    %ijbvect(:,:,i)=ijb;
    condnumber2(k)=cond(ijb)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%
    vectx=[vectx x_center];
    vecty=[vecty y_center];
    vectz=[vectz z_center];
    scatter3(vectx,vecty,vectz)
    
    
    
    xlabel('x-axis')
    ylabel('y-axis')
    zlabel('z-axis')
    
    view(vid);      % select view
    pause(0.1)


    
    Fvideo(k) = getframe(gcf) ;
   frame = Fvideo(k) ; 
   writeVideo(writerObj, frame);

    
end


close(writerObj);




%%
figure  % plot servo angles
plot(condnumber2)
title('Condition Number')
xlabel('Time (s)')
ylabel('Condition Number')

%% Inverse of the conddition number
condnumber2inv=1./condnumber2;
figure  %  
plot(condnumber2inv)
title('Inverse of the Condition Number')
xlabel('Time (s)')
ylabel('Inverse of the Condition Number')
