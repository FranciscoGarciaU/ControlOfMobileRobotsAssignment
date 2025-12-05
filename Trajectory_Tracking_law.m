function [v, w] = Trajectory_Tracking_law(x, y, theta, xr, yr, xdr, ydr, xddr, yddr, ...
                                         Ts, kp1, kp2, kd1, kd2, controller_type, b)
    % Trajectory_Tracking_law - Implements both controllers
    % Inputs:
    %   x, y, theta - current robot state
    %   xr, yr - reference position
    %   xdr, ydr - reference velocity
    %   xddr, yddr - reference acceleration
    %   Ts - sampling time
    %   kp1, kp2, kd1, kd2 - controller gains
    %   controller_type - 1: KES (Kinematic Error Stabilization)
    %                     2: FL-PD (Feedback Linearization PD)
    %   b - distance ahead for output point (only used for FL-PD)
    % Outputs:
    %   v, w - linear and angular velocity commands
    
    if nargin < 16
        b = 0.1; % Default distance for feedback linearization
    end
    
    if controller_type == 1
        % KES Controller (Kinematic Error Stabilization)
        [v, w] = KES_controller(x, y, theta, xr, yr, xdr, ydr, xddr, yddr, ...
                               Ts, kp1, kp2, kd1, kd2);
    else
        % FL-PD Controller (Feedback Linearization PD)
        theta_ref = atan2(ydr, xdr);
        v_ref = sqrt(xdr^2 + ydr^2);
        w_ref = (xdr*yddr - ydr*xddr) / (xdr^2 + ydr^2);
        [v, w] = FL_PD_controller(x, y, theta, xr, yr, theta_ref, v_ref, w_ref, ...
                                 Ts, kp1, kp2, kd1, kd2, b);
    end
end

function [v, w] = KES_controller(x, y, theta, xr, yr, xdr, ydr, xddr, yddr, ...
                                Ts, kp1, kp2, kd1, kd2)
    % KES_CONTROLLER - Kinematic Error Stabilization Controller
    % Implements the controller from De Luca et al.
    % Based on "Trajectory tracking control of a mobile robot" methodology
    
    % Transform position error to robot frame
    R = [cos(theta), sin(theta); -sin(theta), cos(theta)];
    e_global = [xr - x; yr - y];
    e_robot = R * e_global;
    
    ex = e_robot(1);
    ey = e_robot(2);
    
    % Orientation error
    theta_ref = atan2(ydr, xdr);
    e_theta = theta_ref - theta;
    
    % Wrap orientation error to [-pi, pi]
    while e_theta > pi
        e_theta = e_theta - 2*pi;
    end
    while e_theta < -pi
        e_theta = e_theta + 2*pi;
    end
    
    % Reference velocities
    v_ref = sqrt(xdr^2 + ydr^2);
    w_ref = (xdr*yddr - ydr*xddr) / (xdr^2 + ydr^2);
    
    % KES control law
    v = v_ref * cos(e_theta) + kp1 * ex;
    
    % Handle singularity at e_theta = 0
    if abs(e_theta) < 1e-6
        sinc_term = 1;
    else
        sinc_term = sin(e_theta) / e_theta;
    end
    
    w = w_ref + kp2 * v_ref * sinc_term * ey + kd2 * e_theta;
    
    % Add derivative term for position error
    persistent prev_ex prev_time
    if isempty(prev_ex)
        prev_ex = ex;
        prev_time = 0;
    end
    
    if kd1 > 0
        dt = Ts - prev_time;
        if dt > 0
            v = v + kd1 * (ex - prev_ex) / dt;
        end
    end
    
    % Update persistent variables
    prev_ex = ex;
    prev_time = Ts;
end

function [v, w] = FL_PD_controller(x, y, theta, xr, yr, theta_ref, v_ref, w_ref, ...
                                  Ts, kp1, kp2, kd1, kd2, b)
    % FL_PD_CONTROLLER - Feedback Linearization PD Controller
    % Implements the feedback linearization method
    % Uses an output point located distance 'b' ahead of the robot
    
    % 1. Compute output point z (b ahead of robot)
    z = [x + b * cos(theta);
         y + b * sin(theta)];
    
    % 2. Compute reference output point zr
    zr = [xr + b * cos(theta_ref);
          yr + b * sin(theta_ref)];
    
    % 3. Compute transformation matrix T_n(theta)
    T_n = [cos(theta), -b*sin(theta);
           sin(theta),  b*cos(theta)];
    
    % 4. Compute reference velocity in z coordinates (feedforward)
    zr_dot = [v_ref * cos(theta_ref) - b * sin(theta_ref) * w_ref;
              v_ref * sin(theta_ref) + b * cos(theta_ref) * w_ref];
    
    % 5. Compute error and error derivative
    e_z = z - zr;
    
    persistent prev_e_z prev_time_fl
    if isempty(prev_e_z)
        prev_e_z = e_z;
        prev_time_fl = 0;
    end
    
    % Approximate derivative
    dt = Ts - prev_time_fl;
    if dt > 0
        e_z_dot = (e_z - prev_e_z) / dt;
    else
        e_z_dot = [0; 0];
    end
    
    % 6. PD control in linearized coordinates
    Kp = diag([kp1, kp2]);
    Kd = diag([kd1, kd2]);
    
    u_lin = -Kp * e_z - Kd * e_z_dot + zr_dot;
    
    % 7. Transform back to physical inputs
    % T_n is always invertible for b > 0
    T_n_inv = (1/b) * [b*cos(theta), b*sin(theta);
                      -sin(theta), cos(theta)];
    
    vw = T_n_inv * u_lin;
    v = vw(1);
    w = vw(2);
    
    % Update persistent variables
    prev_e_z = e_z;
    prev_time_fl = Ts;
end