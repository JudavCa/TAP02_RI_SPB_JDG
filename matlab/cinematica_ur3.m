clear all
clc
close all
%% Direct Kinematic Model
% DH Matrix - DH=[a_ij alpha_ij s_j theta_j]
 q = [1.570105;-0.552846; 0.567845;-1.585795;-1.570796;-0.000692];
offset_theta = [0; pi; 0; 0; pi; pi];
theta = offset_theta + q;

DH=[   0         deg2rad(0)    0.1519   theta(1);
       0         deg2rad(-90)  0        theta(2);
      -0.24365   deg2rad(0)    0        theta(3);
      -0.21325   deg2rad(0)    0.11235  theta(4);
       0         deg2rad(90)   0.08535  theta(5);
       0         deg2rad(90)   0.0819   theta(6)   
       ];
frame=size(DH,1);
%Reconstruye la matriz homogenea 4*4
for i=1:frame
    T_ij(:,:,i)=T_DH(DH(i,1),DH(i,2),DH(i,3),DH(i,4)); %MARCO LOCAL
end
T(:,:,1)=T_ij(:,:,1);
for i=1:frame-1
    T(:,:,i+1)=T(:,:,i)*T_ij(:,:,i+1);
end
T
%% Graph
% Define the Fixed Frame
hf=figure(1);
set(hf,'position',[445   275   563   628])
% Unit vectors scale factor(visual)
L=0.1;
h_ejexF=plot3([0 L],[0 0],[0 0],'r','linewidth',2.5);
hold on
h_ejeyF=plot3([0 0],[0 L],[0 0],'g','linewidth',2.5);
hold on
h_ejezF=plot3([0 0],[0 0],[0 L],'b','linewidth',2.5);
hold on
axis equal
%axis([-50 50 -50 50 -50 60])
xlabel('X [-]')
ylabel('Y [-]')
zlabel('Z [-]')
view(27,37)
% Mobile Coordinate System Based on DoF
for i=1:frame
    h_ejexM(i,1)=plot3([0 0],[0 0],[0 0],'r','linewidth',2);
    hold on
    h_ejeyM(i,1)=plot3([0 0],[0 0],[0 0],'g','linewidth',2);
    hold on
    h_ejezM(i,1)=plot3([0 0],[0 0],[0 0],'b','linewidth',2);
end
% Points Matrix on {j}
Mp=[0,0,0,1;
    L,0,0,1;
    0,L,0,1;
    0,0,L,1];
% Points Matrix on {0}
for k=1:frame
    for i=1:4
        Mp_T(i,:,k)=(T(:,:,k)*Mp(i,:)')'; % MARCO GLOBAL
    end
end
% Plot each Coordinate System in the Global Frame
for k=1:frame
    set(h_ejexM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(2,1,k)],...
        'ydata',[Mp_T(1,2,k) Mp_T(2,2,k)],...
        'zdata',[Mp_T(1,3,k) Mp_T(2,3,k)])
    set(h_ejeyM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(3,1,k)],...
        'ydata',[Mp_T(1,2,k) Mp_T(3,2,k)],...
        'zdata',[Mp_T(1,3,k) Mp_T(3,3,k)])
    set(h_ejezM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(4,1,k)],...
        'ydata',[Mp_T(1,2,k) Mp_T(4,2,k)],...
        'zdata',[Mp_T(1,3,k) Mp_T(4,3,k)])
end
%% Transform tipo Denavit-Hartenberg
function T_ij=T_DH(a_ij,alpha_ij,s_i,theta_i)
T_ij =[               cos(theta_i),              -sin(theta_i),              0,               a_ij;
        cos(alpha_ij)*sin(theta_i), cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -s_i*sin(alpha_ij);
        sin(alpha_ij)*sin(theta_i), sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  s_i*cos(alpha_ij);
                          0,                          0,              0,                  1];
end
%% Jacobiano de velocidades angulares
joint = ['R' 'R' 'R' 'R' 'R' 'R'];
sigma = double(joint == 'R');

Z = zeros(3,frame);           % <-- agregar esto
for k=1:frame
    Z(:,k) = T(1:3, 3, k);    % <-- guardar el eje Z en Z
    Jw(:,k) = sigma(k) * Z(:,k);
end

%% Jacobiano de velocidades lineales
O = zeros(3,frame);
for k=1:frame
    O(:,k) = T(1:3,4,k);
end
On = O(:,frame);

Jv = zeros(3,frame);
for k=1:frame
    if sigma(k)==1
        Jv(:,k) = cross(Z(:,k), On - O(:,k));
    else
        Jv(:,k) = Z(:,k);
    end
end
%% Jacobiano
J = [Jv; Jw]

%% Velocidad del efector final
 q_dot = [-0.000502; 0.140434; 0.06357; 0.008102; 0.0031; 0.0];

V = J * q_dot;  % V es 6x1

v_lineal = V(1:3)   % velocidad lineal del efector [vx;vy;vz]
w_angular = V(4:6)  % velocidad angular del efector [wx;wy;wz]