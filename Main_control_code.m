function [qseq_KES, vwseq_KES, error_KES, qseq_FL, vwseq_FL, error_FL, xr, yr, t, Ts] = Main_control_code()
% MAIN_CONTROL_CODE - Core simulation function for differential drive robot control
% Runs both KES and FL-PD controllers and returns simulation data
%
% Outputs:
%   qseq_KES, vwseq_KES, error_KES - KES controller data
%   qseq_FL, vwseq_FL, error_FL - FL-PD controller data
%   xr, yr - Reference trajectory
%   t - Time vector
%   Ts - Sampling time

% Clear persistent variables
clear DiffDrive;

%% ==================== SYSTEM PARAMETERS ====================
r = 0.04445;    % Wheel radius [m]
d = 0.393;      % Distance between wheels [m]
Ts = 0.15;      % Sampling time [s]

%% ==================== TRAJECTORY GENERATION ====================
fprintf('Generating reference trajectory...\n');
[xr, yr, thetar, vr, wr, xdr, ydr, xddr, yddr, t] = generate_reference_trajectory(Ts);

% Initial conditions
x0 = 0; y0 = 0; theta0 = 0;
q0 = [x0; y0; theta0];

% Actuator constraints
wrmax = 10;     % Maximum wheel speed [rad/s]
wlmax = wrmax;  % Symmetric constraints

%% ==================== KES CONTROLLER SIMULATION ====================
fprintf('\nRunning KES Controller (Kinematic Error Stabilization)...\n');

% KES controller gains
kp1_kes = 11.5;   
kp2_kes = 9.5;    
kd1_kes = 0.68;    
kd2_kes = 0.53;   
% Run simulation
[qseq_KES, vwseq_KES, error_KES] = simulate_controller(...
    q0, xr, yr, thetar, vr, wr, xdr, ydr, xddr, yddr, ...
    wrmax, wlmax, r, d, Ts, ...
    kp1_kes, kp2_kes, kd1_kes, kd2_kes, 1);  % controller_type = 1 for KES

fprintf('   Final tracking error: %.4f m\n', error_KES(end));

%% ==================== FL-PD CONTROLLER SIMULATION ====================
fprintf('\nRunning FL-PD Controller (Feedback Linearization PD)...\n');

% FL-PD controller gains
kp1_fl = 1.5; kp2_fl = 1.5; kd1_fl = 0.5; kd2_fl = 0.5;
b = 0.1;  % Distance ahead for output point

% Run simulation
[qseq_FL, vwseq_FL, error_FL] = simulate_controller(...
    q0, xr, yr, thetar, vr, wr, xdr, ydr, xddr, yddr, ...
    wrmax, wlmax, r, d, Ts, ...
    kp1_fl, kp2_fl, kd1_fl, kd2_fl, 2, b);  % controller_type = 2 for FL-PD

fprintf('   Final tracking error: %.4f m\n', error_FL(end));

fprintf('\nMain control code execution complete.\n');
end

%% ==================== HELPER FUNCTIONS ====================

function [qseq, vwseq, error_seq] = simulate_controller(q0, xr, yr, thetar, vr, wr, ...
    xdr, ydr, xddr, yddr, wrmax, wlmax, r, d, Ts, ...
    kp1, kp2, kd1, kd2, controller_type, b)
    % SIMULATE_CONTROLLER - Simulates a single controller
    
    % Initialize storage variables
    qseq = q0;
    vwseq = [];
    error_seq = [];
    
    % Handle optional b parameter for FL-PD
    if nargin < 20
        b = 0.1;  % Default value
    end
    
    % Main simulation loop
    for k = 1:length(xr)-1
        % Current state
        q_current = qseq(:, end);
        x = q_current(1);
        y = q_current(2);
        theta = q_current(3);
        
        % Compute control command
        if controller_type == 1  % KES Controller
            [v, w] = Trajectory_Tracking_law(x, y, theta, xr(k), yr(k), ...
                xdr(k), ydr(k), xddr(k), yddr(k), Ts, kp1, kp2, kd1, kd2, 1);
        else  % FL-PD Controller
            [v, w] = Trajectory_Tracking_law(x, y, theta, xr(k), yr(k), ...
                xdr(k), ydr(k), xddr(k), yddr(k), Ts, kp1, kp2, kd1, kd2, 2, b);
        end
        
        % Apply actuator constraints
        [v, w] = unicycle_saturation(wrmax, wlmax, v, w, r, d);
        
        % Store velocity commands
        vwseq = [vwseq, [v; w]];
        
        % Compute tracking error
        current_error = sqrt((x - xr(k))^2 + (y - yr(k))^2);
        error_seq = [error_seq, current_error];
        
        % Propagate dynamics using Euler integration
        x_next = x + Ts * v * cos(theta);
        y_next = y + Ts * v * sin(theta);
        theta_next = theta + Ts * w;
        
        % Wrap orientation to [-π, π]
        theta_next = atan2(sin(theta_next), cos(theta_next));
        
        % Store next state
        qseq = [qseq, [x_next; y_next; theta_next]];
    end
end

function [xr, yr, thetar, vr, wr, xdr, ydr, xddr, yddr, t] = generate_reference_trajectory(Ts)
    % GENERATE_REFERENCE_TRAJECTORY - Creates reference trajectory
    
    % Time vector
    t = 0:Ts:2*pi*4*2;
    
    % Trajectory parameters
    eta = 1;
    alpha = 4;
    
    % Position
    xr = eta * sin(t/alpha);
    yr = eta * sin(t/(2*alpha));
    
    % Velocity
    xdr = eta * cos(t/alpha) * (1/alpha);
    ydr = eta * cos(t/(2*alpha)) * (1/(2*alpha));
    
    % Acceleration
    xddr = -eta * sin(t/alpha) * (1/alpha)^2;
    yddr = -eta * sin(t/(2*alpha)) * (1/(2*alpha))^2;
    
    % Orientation
    thetar = atan2(ydr, xdr);
    
    % Fix orientation discontinuities
    thetar_diff = diff(thetar);
    for i = 1:length(thetar_diff)
        if thetar_diff(i) < -6
            i1 = i+1;
        elseif thetar_diff(i) > 6
            i2 = i;
        end
    end
    if exist('i1', 'var') && exist('i2', 'var')
        thetar(i1:i2) = thetar(i1:i2) + 2*pi;
    end
    
    % Linear and angular velocities
    vr = sqrt(xdr.^2 + ydr.^2);
    wr = (yddr .* xdr - xddr .* ydr) ./ (xdr.^2 + ydr.^2);
end