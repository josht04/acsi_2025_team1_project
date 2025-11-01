clear; clc; close all;

% Animation for drone given the state & input vectors

% state_optimal = load("STATE_0.3to0.35.mat").STATE_opt;
% fr = load("FR_0.3to0.35.mat").FR_opt;
% fl = load("FL_0.3to0.35.mat").FL_opt;

state_optimal_unsliced = load("STATE_0.15to0.5.mat").STATE_all;
fr_unsliced = load("FR_0.15to0.5.mat").FR_all;
fl_unsliced = load("FL_0.15to0.5.mat").FL_all;

state_optimal_unsliced(:,:,15)=[];
fr_unsliced(:,15)=[];
fl_unsliced(:,15)=[];

state_optimal = state_optimal_unsliced(:, :, 1:5:36);
fr = fr_unsliced(:, 1:5:36);
fl = fl_unsliced(:, 1:5:36);

% fr/fl are matrix of num-drone columns, and time rows.
fl(end+1,:) = 0.18;
fr(end+1,:) = 0.18;
fl = fl-0.18;
fr = fr-0.18;

time = linspace(0,4,101)';
y = state_optimal(:,1,:);
z = state_optimal(:,2,:);
phi = state_optimal(:,3,:);
theta = state_optimal(:,4,:);
ydot = state_optimal(:,5,:);
zdot = state_optimal(:,6,:);
phidot = state_optimal(:,7,:);
thetadot = state_optimal(:,8,:);

theta = squeeze(theta);
phi = squeeze(phi);
z = squeeze(z);
y = squeeze(y);
thetadot = squeeze(thetadot);
phidot = squeeze(phidot);
zdot = squeeze(zdot);
ydot = squeeze(ydot);

lengths = 0.15:0.05:0.50;

dronePendulumAnimation_fromData(time, y,z, phi, theta, fl, fr, lengths, thetadot, phidot, zdot, ydot);

% animation function
function dronePendulumAnimation_fromData(time, y, z, phi, theta, fl, fr, lengths, thetadot, phidot, zdot, ydot)
    
    drone_r = 0.1;
    pendulum_length_init = 0.3;
    ball_radius = 0.2;
    
    % Check if data vectors have the same length
    if ~isequal(size(y), size(z), size(phi), size(theta), size(fl), size(fr))
        error('All input data vectors must have the same length.');
    end

    % --- Setup Figure and Axes ---
    figure('Name', 'Drone with Pendulum Animation (from Data)', 'Renderer', 'painters');
    ax = gca;
    hold on;
    grid on;
    axis equal;
    
    % Dynamically set axis limits based on data range, with a buffer
    y_min = min(y,[],"all") - pendulum_length_init - 0.5;
    y_max = max(y,[],"all") + pendulum_length_init + 0.5;
    z_min = min(z,[],"all") - pendulum_length_init - 0.2;
    z_max = max(z,[],"all") + pendulum_length_init + 0.5;
    xlim([y_min y_max]);
    ylim([z_min z_max]);
    
    xlabel('Y Position (m)');
    ylabel('Z Position (m)');
    title('Drone with Pendulum');

    % --- Plot Initial State (Get handles to graphical objects) ---

    drone_bodies = [];
    forces = [];
    pendulum_handles = [];
    ball_handles = [];
    
    % Get initial values from the first element of the data vectors
    for i=1:length(lengths)
        pendulum_length = lengths(i);

        y0 = y(1,i);
        z0 = z(1,i);
        phi0 = phi(1,i);
        theta0 = theta(1,i);
        fl0 = fl(1,i);
        fr0 = fr(1,i);
    
        % Drone Body
        drone_left_y0 = y0-drone_r*cos(phi0);
        drone_right_y0 = y0+drone_r*cos(phi0);
        drone_left_z0 = z0-drone_r*sin(phi0);
        drone_right_z0 = z0+drone_r*sin(phi0);
        drone_body = plot(ax, [drone_left_y0, drone_right_y0], [drone_left_z0, drone_right_z0], 'LineWidth',5);
        drone_bodies = [drone_bodies drone_body];
    
        % Forces
        fl_y0 = fl0*sin(phi0);
        fl_z0 = fl0*cos(phi0);
        fr_y0 = fr0*sin(phi0);
        fr_z0 = fr0*cos(phi0);
        
        
        force_L = quiver(drone_left_y0, drone_left_z0, -fl_y0, fl_z0, 3, 'LineWidth',1.5, 'MaxHeadSize',2);
        force_R = quiver(drone_right_y0, drone_right_z0, -fr_y0, fr_z0, 3, 'LineWidth',1.5, 'MaxHeadSize',2);

        forces = [forces force_L force_R];
    
        % Drone Rotors
        % rotor_offset_x = drone_width/2 * 0.7;
        % rotor_radius = 0.1;
        % rotor1_handle = plot(ax, drone_x_0 - rotor_offset_x, drone_y_0 + drone_height/2 + rotor_radius, 'o', ...
        %                      'MarkerSize', 8, 'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerEdgeColor', 'k');
        % rotor2_handle = plot(ax, drone_x_0 + rotor_offset_x, drone_y_0 + drone_height/2 + rotor_radius, 'o', ...
        %                      'MarkerSize', 8, 'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerEdgeColor', 'k');
    
        % Calculate initial ball position
        ball_y0 = y0 + pendulum_length * sin(theta0);
        ball_z0 = z0 - pendulum_length * cos(theta0);
    
        % Pendulum rod and Ball
        pendulum_handle = plot(ax, [y0,ball_y0], [z0, ball_z0], 'r-', 'LineWidth', 2);
        pendulum_handles = [pendulum_handles pendulum_handle];
        ball_handle = plot(ax, ball_y0, ball_z0, 'o', 'MarkerSize', ball_radius*2*10, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');
        ball_handles = [ball_handles ball_handle];
        
    end
    % --- Animation Loop ---

    lost_tension = ones(length(lengths),1);
    for k = 1:length(time)
        for i = 1:length(lengths)

            pendulum_length = lengths(i);

            tension = checkPendulumTension(y(k,i),z(k,i), phi(k,i), theta(k,i), thetadot(k,i), phidot(k,i), zdot(k,i), ydot(k,i),lengths(i), fl(k,i), fr(k,i));
            if tension < 0 && k > 80
                lost_tension(i) = -1;
                set(drone_bodies(i),'Color', [0.25 0.25 0.25]);
                set(forces(i*2-1),'Color', [0.25 0.25 0.25]);
                set(forces(i*2),'Color', [0.25 0.25 0.25]);
                set(pendulum_handles(i),'Color', [0.25 0.25 0.25]);
                set(ball_handles(i),'Color', [0.25 0.25 0.25]);
            end

            if lost_tension(i)<0
                continue
            end
            % --- Get current state from pre-calculated vectors ---
            % NO SIMULATION HERE - just reading the data for frame 'k'
            drone_y = y(k,i);
            drone_z = z(k,i);
            drone_phi = phi(k,i);
            drone_theta = theta(k,i);
            drone_fl = fl(k,i);
            drone_fr = fr(k,i);
            
            % Attachment point follows the drone
            
            % --- Update Graphical Objects ---
            % Update drone body position
            drone_left_y = drone_y-drone_r*cos(drone_phi);
            drone_right_y = drone_y+drone_r*cos(drone_phi);
            drone_left_z = drone_z-drone_r*sin(drone_phi);
            drone_right_z = drone_z+drone_r*sin(drone_phi);
            
            set(drone_bodies(i), 'XData', [drone_left_y, drone_right_y],'YData', [drone_left_z, drone_right_z])
         
            % Update forces
            fl_y = drone_fl*sin(drone_phi);
            fl_z = drone_fl*cos(drone_phi);
            fr_y = drone_fr*sin(drone_phi);
            fr_z = drone_fr*cos(drone_phi);   
            
            set(forces(i*2-1), 'XData', drone_left_y, 'YData', drone_left_z, 'UData',-fl_y, 'VData',fl_z);
            set(forces(i*2), 'XData', drone_right_y, 'YData', drone_right_z, 'UData',-fr_y, 'VData',fr_z);
    
            % Calculate new ball position based on data
            ball_y = drone_y + pendulum_length * sin(drone_theta);
            ball_z = drone_z - pendulum_length * cos(drone_theta);
    
            % Update pendulum rod
            set(pendulum_handles(i), 'XData', [drone_y, ball_y], 'YData', [drone_z, ball_z]);
    
            % Update ball
            set(ball_handles(i), 'XData', ball_y, 'YData', ball_z);
    
            % --- Redraw and Pause ---
            drawnow;
        end
    end
    hold off;
end

function tension = checkPendulumTension(y,z, phi, theta, theta_d, phi_d, z_d, y_d,L, Fl, Fr)
    mq = 0.029; % kg
    mb = 0.01; % kg
    ms = 0.0025; % kg
    Ixx = 6.410179e-06; % kg m2
    g = 9.81; % kg/m2
    r = 0.05665/2; % m

    Ip = ms*L^2/3+mb*L^2;
    Lcg = L*(ms/2+mb)/(ms+mb);
    
    % A*x_dd = B
    
    % Dynamics derived from the pdf
    A = [ms+mb+mq 0 (ms+mb)*Lcg*cos(phi+theta) (ms+mb)*Lcg*cos(phi+theta);
        0 ms+mb+mq (ms+mb)*Lcg*sin(phi+theta) (ms+mb)*Lcg*sin(phi+theta);
        Lcg*cos(phi+theta)*(ms+mb) Lcg*sin(phi+theta)*(ms+mb) Ip Ip;
        0 0 Ixx 0];
    
    B = [-(Fl+Fr)*sin(phi)+(ms+mb)*Lcg*sin(phi+theta)*(phi_d+theta_d)^2 ;
        (Fl+Fr)*cos(phi)-(ms+mb)*Lcg*cos(phi+theta)*(phi_d+theta_d)^2-g*(ms+mb+mq) ;
        -g*Lcg*(ms+mb)*sin(phi+theta) ; 
        (Fr-Fl)*r];
    
    state_dd = A\B;
    y_dd = state_dd(1);
    z_dd = state_dd(2);
    
    tension = theta_d^2*Lcg +(g+z_dd)*cos(phi+theta)-y_dd*sin(phi+theta);
    % tension = 1;
end