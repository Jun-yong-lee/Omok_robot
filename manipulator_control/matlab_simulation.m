%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5DOF Manipulator Project Path Planning
% Department of Robotics, Hoseo University
% Student Number : 20172390
% Name : Lee Seong Yong
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Length unit : cm
% Degree unit : radian
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Manipulator Radius & Theta : Default Y Coordinate(theta = PI/2)
radius = zeros(19,19);theta = zeros(19,19);
% Joint theta
jt_1 = zeros(19,19);jt_2 = zeros(19,19);jt_3 = zeros(19,19);
% end-effector theta(roll : not used)
pitch = zeros(19,19);yaw = zeros(19,19);

%% Manipulator Analysis : Articulated Type(RRR)
% Manipulator Link Length Parameter
% link_1 = 24.13;link_2 = 32.6;link_3 = 24;ejector = 12.15;
link_1 = 24.13;link_2 = 32.6;link_3 = 29;ejector = 12.15;

% Manipulator End-Effector Position Parameter
xd = zeros(19,19);yd = zeros(19,19);zd = zeros(19,19);

% Manipulator Base Position Parameter
init_x = 0.0;init_y = 0.0;
b_radius = 9.5;b_place = 7.5;b_blank = 1.45; % origin b_place = 7.5
b_origin = b_radius+b_place+b_blank;

% Intaglio Parameter
intaglio_height = 1.3;intaglio_square_width_length = 2.05;
cancave_stone_height = 0.8; % usually 0.7~1.2[cm]

%% Radius and Angle of Concave Intersection from Base
for y = 1:19 % y coordinate
    for x = 1:19 % x coordinate
        if x < 10
            radius(y,x) = sqrt((intaglio_square_width_length*(10-x))^2+(intaglio_square_width_length*(19-y)+b_origin)^2);
            theta(y,x) = (pi/2)+atan2((intaglio_square_width_length*(10-x)),(intaglio_square_width_length*(19-y)+b_origin));
        elseif x > 10
            radius(y,x) = sqrt((intaglio_square_width_length*(x-10))^2+(intaglio_square_width_length*(19-y)+b_origin)^2);
            theta(y,x) = (pi/2)-atan2((intaglio_square_width_length*(x-10)),(intaglio_square_width_length*(19-y)+b_origin));
        else
            radius(y,x) = sqrt((intaglio_square_width_length*(19-y)+b_origin)^2);
            theta(y,x) = pi/2;
        end
    end
end
%% figure(1) Manipulator Workspace Simulation
workspace_radius = link_2 + link_3 - 0.000001;
figure(1)
grid on;hold on;
title('Manipulator Workspace');
xlim([-72.5 72.5]);ylim([-72.5 72.5]);zlim([0 145]);
xlabel('x axis(cm)');ylabel('y axis(cm)');zlabel('z axis(cm)');
for ro_theta = -6*pi/50:pi/50:pi/2
    for ws_theta = 0:pi/50:2*pi
        xd = init_x+(workspace_radius - (workspace_radius - workspace_radius*cos(ro_theta)))*cos(ws_theta);
        yd = init_y+(workspace_radius - (workspace_radius - workspace_radius*cos(ro_theta)))*sin(ws_theta);
        zd = link_1+(workspace_radius*sin(ro_theta));
    
        d = sqrt(xd^2+yd^2+(zd-link_1)^2);
        D = (d^2-link_2^2-link_3^2)/(2*link_2*link_3);
        wjt_3 = atan2(-sqrt(1-D^2),D);
        Alpha = atan2(zd-link_1,sqrt(xd^2+yd^2));
        E = (link_2^2+d^2-link_3^2)/(2*link_2*d);
        Beta = atan2(sqrt(1-E^2),E);
        wjt_2 = Alpha+Beta;
        wjt_1 = atan2(yd,xd);
    
        % Forward Kinematics
        rrx = link_2*cos(wjt_1);
        rry = link_2*sin(wjt_1);
        rrz = link_1+link_2*sin(wjt_2);
        x_coordinate = cos(wjt_1)*(link_2*cos(wjt_2)+link_3*(cos(wjt_2+wjt_3)));
        y_coordinate = sin(wjt_1)*(link_2*cos(wjt_2)+link_3*(cos(wjt_2+wjt_3)));
        z_coordinate = link_1+link_2*sin(wjt_2)+link_3*(sin(wjt_2+wjt_3));
        
%         % Manipulator Body
%         rx = [0,0,rrx,x_coordinate];
%         ry = [0,0,rry,y_coordinate];
%         rz = [0,link_1,rrz,z_coordinate];
%         plot3(rx,ry,rz,'k-','LineWidth',1);plot3(rx,ry,rz,'go');
        plot3(x_coordinate,y_coordinate,z_coordinate,'c.'); % Manipulator End-Effector Position Coordinate
%         plot3(xd,yd,zd,'go');
    end
end


for y = 1:19 % y coordinate
    for x = 1:19 % x coordinate
        % circular trajectory & vacuum aspirator length
        xd = init_x+radius(y,x)*cos(theta(y,x));
        yd = init_y+radius(y,x)*sin(theta(y,x));
        zd = ejector+intaglio_height+cancave_stone_height;
        
        % Inverse Kinematics
        d = sqrt(xd^2+yd^2+(zd-link_1)^2);
        D = (d^2-link_2^2-link_3^2)/(2*link_2*link_3);
        jt_3(y,x) = atan2(-sqrt(1-D^2),D);
        Alpha = atan2(zd-link_1,sqrt(xd^2+yd^2));
        E = (link_2^2+d^2-link_3^2)/(2*link_2*d);
        Beta = atan2(sqrt(1-E^2),E);
        jt_2(y,x) = Alpha+Beta;
        jt_1(y,x) = atan2(yd,xd);
        
        % Forward Kinematics
        rrx = link_2*cos(jt_1(y,x));
        rry = link_2*sin(jt_1(y,x));
        rrz = link_1+link_2*sin(jt_2(y, x));
        x_coordinate = cos(jt_1(y,x))*(link_2*cos(jt_2(y, x))+link_3*(cos(jt_2(y, x)+jt_3(y, x))));
        y_coordinate = sin(jt_1(y,x))*(link_2*cos(jt_2(y, x))+link_3*(cos(jt_2(y, x)+jt_3(y, x))));
        z_coordinate = link_1+link_2*sin(jt_2(y, x))+link_3*(sin(jt_2(y, x)+jt_3(y, x)));
        
        % Manipulator Body
        rx = [0,0,rrx,x_coordinate];
        ry = [0,0,rry,y_coordinate];
        rz = [0,link_1,rrz,z_coordinate];
        plot3(rx,ry,rz,'k-','LineWidth',1);plot3(rx,ry,rz,'go');
        plot3(x_coordinate,y_coordinate,z_coordinate,'y*'); % Manipulator End-Effector Position Coordinate
        plot3(xd,yd,zd,'go');
        
        % Manipulator End-Effector
        ejector_form = linspace(zd,zd-ejector,10);
        plot3(x_coordinate,y_coordinate,ejector_form,'k.','LineWidth',2);
        plot3(x_coordinate,y_coordinate,ejector_form(:,10),'cv','LineWidth',2);
        % Intaglio Junction
        plot3(x_coordinate,y_coordinate,intaglio_height,'K*');
        plot3(x_coordinate,y_coordinate,intaglio_height,'Ko');
        
        % Manipulator Rotation & Orientation Data Save
        jt_1(y,x) = jt_1(y,x)*(180/pi);
        jt_2(y,x) = jt_2(y,x)*(180/pi);
        jt_3(y,x) = jt_3(y,x)*(180/pi);

        % Inverse Kinematics End-Effector
        if (jt_2(y,x) > 0) && (jt_3(y,x) < 0)
            if (abs(jt_3(y,x))-abs(jt_2(y,x))) > 90
                pitch(y,x) = abs((abs(jt_3(y,x))-abs(jt_2(y,x)))-90);
            elseif (abs(jt_3(y,x))-abs(jt_2(y,x))) < 90
                pitch(y,x) = -abs(90-(abs(jt_3(y,x))-abs(jt_2(y,x))));
            end
        elseif (jt_2(y,x) < 0) && (jt_3(y,x) < 0) && ((abs(jt_2(y,x)) + abs(jt_3(y,x))) < 90)
            pitch(y,x) = -abs(90-(abs(jt_2(y,x))+abs(jt_3(y,x))));
        end
    end
end

intaglio_position = b_radius+b_place;
% lntaglio Vertex Coordinates
intaglio_vertex=[-20 intaglio_position 0 ; 20 intaglio_position 0; 20 (intaglio_position+40) 0; -20 (intaglio_position+40) 0; -20 ...
                intaglio_position intaglio_height; 20 intaglio_position intaglio_height; 20 intaglio_position+40 intaglio_height; -20 (intaglio_position+40) intaglio_height];
% Intaglio Cross-Section
intaglio_face=[1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8];
% Intaglio Cross-Sectional Area Color
intaglio_face_color=[0.93 0.69 0.13; 0.93 0.69 0.13 ; 0.93 0.69 0.13; 0.93 0.69 0.13; 0.93 0.69 0.13; 0.93 0.69 0.13];
patch('Vertices',intaglio_vertex,'faces',intaglio_face,'FaceVertexCData',intaglio_face_color,'Facecolor','flat');

% Manipulator Base Position Coordinates
cylinder_N = 300; % Uniformly spaced points to create a cylinder
cylinder_height = 5;
[cx,cy,cz] = cylinder(b_radius,cylinder_N);
surf(cx,cy,cz*cylinder_height);

%% Manipulator 3D Plot Relative Coordinate Simulation
figure(2)
grid on;hold on;
title('Circular Trajectory for Three Link Manipulator(Upper Arm)');
xlabel('x axis(cm)');ylabel('y axis(cm)');zlabel('z axis(cm)');
xlim([-40 40]);ylim([-20 60]);zlim([0 80]);
for y = 1:19 % y coordinate
    for x = 1:19 % x coordinate
        % circular trajectory & vacuum aspirator length
        xd = init_x+radius(y,x)*cos(theta(y,x));
        yd = init_y+radius(y,x)*sin(theta(y,x));
        zd = ejector+intaglio_height+cancave_stone_height;
        
        % Inverse Kinematics
        d = sqrt(xd^2+yd^2+(zd-link_1)^2);
        D = (d^2-link_2^2-link_3^2)/(2*link_2*link_3);
        jt_3(y,x) = atan2(-sqrt(1-D^2),D);
        Alpha = atan2(zd-link_1,sqrt(xd^2+yd^2));
        E = (link_2^2+d^2-link_3^2)/(2*link_2*d);
        Beta = atan2(sqrt(1-E^2),E);
        jt_2(y,x) = Alpha+Beta;
        jt_1(y,x) = atan2(yd,xd);
        
        % Forward Kinematics
        rrx = link_2*cos(jt_1(y,x));
        rry = link_2*sin(jt_1(y,x));
        rrz = link_1+link_2*sin(jt_2(y, x));
        x_coordinate = cos(jt_1(y,x))*(link_2*cos(jt_2(y, x))+link_3*(cos(jt_2(y, x)+jt_3(y, x))));
        y_coordinate = sin(jt_1(y,x))*(link_2*cos(jt_2(y, x))+link_3*(cos(jt_2(y, x)+jt_3(y, x))));
        z_coordinate = link_1+link_2*sin(jt_2(y, x))+link_3*(sin(jt_2(y, x)+jt_3(y, x)));
        
        % Manipulator Body
        rx = [0,0,rrx,x_coordinate];
        ry = [0,0,rry,y_coordinate];
        rz = [0,link_1,rrz,z_coordinate];
        plot3(rx,ry,rz,'k-','LineWidth',1);plot3(rx,ry,rz,'go');
        plot3(x_coordinate,y_coordinate,z_coordinate,'y*'); % Manipulator End-Effector Position Coordinate
        plot3(xd,yd,zd,'go');
        
        % Manipulator End-Effector
        ejector_form = linspace(zd,zd-ejector,10);
        plot3(x_coordinate,y_coordinate,ejector_form,'k.','LineWidth',2);
        plot3(x_coordinate,y_coordinate,ejector_form(:,10),'cv','LineWidth',2);
        % Intaglio Junction
        plot3(x_coordinate,y_coordinate,intaglio_height,'K*');
        plot3(x_coordinate,y_coordinate,intaglio_height,'Ko');

        % Manipulator Body
        rx = [0,0,rrx,x_coordinate];
        ry = [0,0,rry,y_coordinate];
        rz = [0,link_1,rrz,z_coordinate];
        plot3(rx,ry,rz,'k-','LineWidth',1);plot3(rx,ry,rz,'go');
        plot3(x_coordinate,y_coordinate,z_coordinate,'y*'); % Manipulator End-Effector Position Coordinate
        plot3(xd,yd,zd,'go');
        
        % Manipulator End-Effector
        ejector_form = linspace(zd,zd-ejector,10);
        plot3(x_coordinate,y_coordinate,ejector_form,'k.','LineWidth',2);
        plot3(x_coordinate,y_coordinate,ejector_form(:,10),'cv','LineWidth',2);
        % Intaglio Junction
        plot3(x_coordinate,y_coordinate,intaglio_height,'K*');
        plot3(x_coordinate,y_coordinate,intaglio_height,'Ko');
        
        % Manipulator Rotation & Orientation Data Save
        jt_1(y,x) = jt_1(y,x)*(180/pi);
        jt_2(y,x) = jt_2(y,x)*(180/pi);
        jt_3(y,x) = jt_3(y,x)*(180/pi); 
            
        plot3(x_coordinate,y_coordinate,intaglio_height,'K*');
        plot3(x_coordinate,y_coordinate,intaglio_height,'Ko');
        % Manipulator Rotation & Orientation Data Save
        jt_1(y,x) = jt_1(y,x)*(180/pi);
        jt_2(y,x) = jt_2(y,x)*(180/pi);
        jt_3(y,x) = jt_3(y,x)*(180/pi);
        
        % Inverse Kinematics End-Effector
        if (jt_2(y,x) > 0) && (jt_3(y,x) < 0)
            if (abs(jt_3(y,x))-abs(jt_2(y,x))) > 90
                pitch(y,x) = abs((abs(jt_3(y,x))-abs(jt_2(y,x)))-90);
            elseif (abs(jt_3(y,x))-abs(jt_2(y,x))) < 90
                pitch(y,x) = -abs(90-(abs(jt_3(y,x))-abs(jt_2(y,x))));
            end
        elseif (jt_2(y,x) < 0) && (jt_3(y,x) < 0) && ((abs(jt_2(y,x)) + abs(jt_3(y,x))) < 90)
            pitch(y,x) = -abs(90-(abs(jt_2(y,x))+abs(jt_3(y,x))));
        end
    end
end

%% intaglio form 3D Plot
intaglio_position = b_radius+b_place;
% lntaglio Vertex Coordinates
intaglio_vertex=[-20 intaglio_position 0 ; 20 intaglio_position 0; 20 (intaglio_position+40) 0; -20 (intaglio_position+40) 0; -20 ...
                intaglio_position intaglio_height; 20 intaglio_position intaglio_height; 20 intaglio_position+40 intaglio_height; -20 (intaglio_position+40) intaglio_height];
% Intaglio Cross-Section
intaglio_face=[1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8];
% Intaglio Cross-Sectional Area Color
intaglio_face_color=[0.93 0.69 0.13; 0.93 0.69 0.13 ; 0.93 0.69 0.13; 0.93 0.69 0.13; 0.93 0.69 0.13; 0.93 0.69 0.13];
patch('Vertices',intaglio_vertex,'faces',intaglio_face,'FaceVertexCData',intaglio_face_color,'Facecolor','flat');

% Manipulator Base Position Coordinates
cylinder_N = 300; % Uniformly spaced points to create a cylinder
cylinder_height = 5;
[cx,cy,cz] = cylinder(b_radius,cylinder_N);
surf(cx,cy,cz*cylinder_height);

%% Path Planning Graph intaglio form 3D Plot
figure(3)
intaglio_position = b_radius+b_place;
% lntaglio Vertex Coordinates
intaglio_vertex=[-20 intaglio_position 0 ; 20 intaglio_position 0; 20 (intaglio_position+40) 0; -20 (intaglio_position+40) 0; -20 ...
                intaglio_position intaglio_height; 20 intaglio_position intaglio_height; 20 intaglio_position+40 intaglio_height; -20 (intaglio_position+40) intaglio_height];
% Intaglio Cross-Section
intaglio_face=[1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8];
% Intaglio Cross-Sectional Area Color
intaglio_face_color=[0.93 0.69 0.13; 0.93 0.69 0.13 ; 0.93 0.69 0.13; 0.93 0.69 0.13; 0.93 0.69 0.13; 0.93 0.69 0.13];
patch('Vertices',intaglio_vertex,'faces',intaglio_face,'FaceVertexCData',intaglio_face_color,'Facecolor','flat');

% Manipulator Base Position Coordinates
[cx,cy,cz] = cylinder(b_radius,cylinder_N);
surf(cx,cy,cz*cylinder_height);

% Select X,Y Coordinate
% input_x = 1;input_y = 1;

% Reference X,Y Coordinate(Reference X,Y Position of Manipulator End-Effector Vacuum Ejector)
reference_x = 10; reference_y = 19;

while(-1)
    coordinate_data = input('x, y coordinate data = ','s');[x_input_data,y_input_data] = strtok(coordinate_data);
    input_x = str2double(x_input_data);input_y = str2double(y_input_data);
    
    % Manipulator Path Planning Simulation : Path Planning Information Parameter
    path_limit_height = 5;
    path_resolution = 5*sqrt(abs(input_x-10)^2+(19-input_y)^2);
    if(input_x == 10)&&(input_y == 19), path_resolution = 1;
    end
    
    if((input_x < 1)||(input_x > 19))||((input_y < 1)||(input_y > 19)), disp('Please Enter the Correct Coordinates!!');break;
    else
        figure(3)
        % Error From Reference Position
        error_radius = abs(radius(input_y,input_x)-radius(reference_y,reference_x));
        error_theta = abs(theta(input_y,input_x)-theta(reference_y,reference_x));
        for rev = 1:path_resolution
            if input_x < 10
                xd = init_x+(radius(reference_y,reference_x)+rev*(error_radius/path_resolution))*cos(theta(reference_y,reference_x)+rev*(error_theta/path_resolution));
                yd = init_y+(radius(reference_y,reference_x)+rev*(error_radius/path_resolution))*sin(theta(reference_y,reference_x)+rev*(error_theta/path_resolution));
            elseif input_x > 10
            xd = init_x+(radius(reference_y,reference_x)+rev*(error_radius/path_resolution))*cos(theta(reference_y,reference_x)-rev*(error_theta/path_resolution));
            yd = init_y+(radius(reference_y,reference_x)+rev*(error_radius/path_resolution))*sin(theta(reference_y,reference_x)-rev*(error_theta/path_resolution));
            else
                xd = init_x+(radius(reference_y,reference_x)+rev*(error_radius/path_resolution))*cos(theta(reference_y,reference_x));
                yd = init_y+(radius(reference_y,reference_x)+rev*(error_radius/path_resolution))*sin(theta(reference_y,reference_x));
            end
    
            if rev <= (path_resolution/2)
                zd = ejector+intaglio_height+cancave_stone_height+rev*(path_limit_height/(path_resolution/2)); 
            else
                zd = ejector+intaglio_height+cancave_stone_height+path_limit_height-(rev-(path_resolution/2))*(path_limit_height/(path_resolution/2));
            end
    
            % Inverse Kinematics
            d = sqrt(xd^2+yd^2+(zd-link_1)^2);
            D = (d^2-link_2^2-link_3^2)/(2*link_2*link_3);
            pjt_3 = atan2(-sqrt(1-D^2),D);
            Alpha = atan2(zd-link_1,sqrt(xd^2+yd^2));
            E = (link_2^2+d^2-link_3^2)/(2*link_2*d);
            Beta = atan2(sqrt(1-E^2),E);
            pjt_2 = Alpha+Beta;
            pjt_1 = atan2(yd,xd);
        
            % Forward Kinematics
            rrx = link_2*cos(pjt_1);
            rry = link_2*sin(pjt_1);
            rrz = link_1+link_2*sin(pjt_2);
            x_coordinate = cos(pjt_1)*(link_2*cos(pjt_2)+link_3*(cos(pjt_2+pjt_3)));
            y_coordinate = sin(pjt_1)*(link_2*cos(pjt_2)+link_3*(cos(pjt_2+pjt_3)));
            z_coordinate = link_1+link_2*sin(pjt_2)+link_3*(sin(pjt_2+pjt_3));
            
            % Manipulator Body
            rx = [0,0,rrx,x_coordinate];
            ry = [0,0,rry,y_coordinate];
            rz = [0,link_1,rrz,z_coordinate];
            plot3(rx,ry,rz,'k-','LineWidth',3);
            hold on;plot3(rx,ry,rz,'ro','LineWidth',2);
            title('4DOF Manipulator Path Planning Simulation');
            xlabel('x axis(cm)');ylabel('y axis(cm)');zlabel('z axis(cm)');
            xlim([-40 40]);ylim([-20 60]);zlim([0 80]);
            plot3(x_coordinate,y_coordinate,z_coordinate,'y*','LineWidth',2); % Manipulator End-Effector Position Coordinate
            plot3(xd,yd,zd,'go','LineWidth',3);
            
            % Manipulator End-Effector
            ejector_form = linspace(zd,zd-ejector,10);
            plot3(x_coordinate,y_coordinate,ejector_form,'k.','LineWidth',2);
            plot3(x_coordinate,y_coordinate,ejector_form(:,10),'cv','LineWidth',2);
            patch('Vertices',intaglio_vertex,'faces',intaglio_face,'FaceVertexCData',intaglio_face_color,'Facecolor','flat');
            surf(cx,cy,cz*cylinder_height);
            grid on;hold off;
            pause(0.01);
        end

        hold on;
        for y = 1:19
            for x = 1:19
                % Intaglio Junction
                xd = init_x+radius(y,x)*cos(theta(y,x));
                yd = init_y+radius(y,x)*sin(theta(y,x));
                plot3(xd,yd,intaglio_height,'K*');
                plot3(xd,yd,intaglio_height,'Ko');
            end
        end
    end
    % Display Joint Rotation & Pitch Orientation
    pjt_1 = pjt_1*(180/pi);pjt_2 = pjt_2*(180/pi);pjt_3 = pjt_3*(180/pi);
    % Inverse Kinematics End-Effector
    if (pjt_2 > 0) && (pjt_3 < 0)
        if (abs(pjt_3)-abs(pjt_2)) > 90
            ppitch = abs((abs(pjt_3)-abs(pjt_2))-90);
        elseif (abs(pjt_3)-abs(pjt_2)) < 90
            ppitch = -abs(90-(abs(pjt_3)-abs(pjt_2)));
        end
    elseif (pjt_2 < 0) && (pjt_3 < 0) && ((abs(pjt_2) + abs(pjt_3)) < 90)
        ppitch = -abs(90-(abs(pjt_2)+abs(pjt_3)));
    end
    disp('Rotation and Orientation of Manipulator based on Relative Coordinate System');
    s0 = sprintf('X Coordinate = %d, Y Coordinate = %d',input_x, input_y);
    s1 = sprintf('Joint(1) = %f', pjt_1);
    s2 = sprintf('Joint(2) = %f', pjt_2);
    s3 = sprintf('Joint(3) = %f', pjt_3);
    s4 = sprintf('Pitch = %f', ppitch);
    disp(s0);disp(s1);disp(s2);disp(s3);disp(s4);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% End
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%