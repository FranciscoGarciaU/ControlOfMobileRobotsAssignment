function qpunto = DiffDrive(t, q, v, w)
    % DIFFDRIVE - Differential drive robot kinematics
    % Inputs:
    %   t - time (not used, for ode45 compatibility)
    %   q - state vector [x; y; theta]
    %   v - linear velocity
    %   w - angular velocity
    % Output:
    %   qpunto - state derivative
    
    persistent stored_v stored_w
    
    % Store v and w if provided
    if nargin == 4
        stored_v = v;
        stored_w = w;
        qpunto = [];
        return;
    end
    
    % Use stored values
    if nargin == 2
        if isempty(stored_v) || isempty(stored_w)
            error('v and w not initialized. Call DiffDrive([], [], v, w) first.');
        end
        v = stored_v;
        w = stored_w;
    end
    
    % Unicycle kinematics
    x = q(1);
    y = q(2);
    theta = q(3);
    
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = w;
    
    qpunto = [x_dot; y_dot; theta_dot];
end