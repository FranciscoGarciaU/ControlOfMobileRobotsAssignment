function [v, w] = unicycle_saturation(wrmax, wlmax, v, w, r, d)
    % UNICYCLE_SATURATION - Applies wheel speed constraints to unicycle inputs
    % Inputs:
    %   wrmax, wlmax - maximum wheel speeds (rad/s)
    %   v, w - desired linear and angular velocities
    %   r - wheel radius
    %   d - distance between wheels
    % Outputs:
    %   v, w - saturated velocities that respect wheel constraints
    
    % Compute wheel velocities
    w_right = (v + (d/2)*w) / r;
    w_left = (v - (d/2)*w) / r;
    
    % Check constraints
    if abs(w_right) > wrmax
        w_right = sign(w_right) * wrmax;
    end
    
    if abs(w_left) > wlmax
        w_left = sign(w_left) * wlmax;
    end
    
    % Convert back to v, w
    v = (r/2) * (w_right + w_left);
    w = (r/d) * (w_right - w_left);
end