function [q_ref_L, q_ref_R, t_sim] = bezier_curve_jump(initial_pos, jump_height)
% This function generates bezier curve trajectories for leg motion during jumping.

% Things to work on:
% 15 point bezier - 3 curve stitching
% smooth transition from stance to crouch
% add end stance bezier

% Defining time range for 3 curves
p.t_first_pt = 0.1;
p.t_second_pt = 0.17;
p.t_third_pt = 0.2;
p.t_fourth_pt = 0.25;

% Define parameters
% Period of the gait cycles for 3 bezier curves
p.gait_period_first = p.t_second_pt - p.t_first_pt; % 0.07
p.gait_period_second = p.t_third_pt - p.t_second_pt; % 0.03
p.gait_period_third = p.t_fourth_pt - p.t_third_pt; % 0.05
p.gait_period_total = p.t_fourth_pt - p.t_first_pt; % 0.15

p.t_start = 0.1;              % Start time
gait_length = 2;              % Number of gait cycles
p.t_end = gait_length * p.gait_period_total + p.t_start; % End time of the simulation
p.dt = 0.005;                 % Time step
t_sim = 0:p.dt:p.t_end;       % Array of time stamps
N = length(t_sim);            % Number of time steps
flag = 0;

% Initialize bezier curves for left and right legs
q_ref_L = zeros(3, N);
q_ref_R = zeros(3, N);

% Generate bezier curves for initial leg positions
BL = bezier_curve_jump_initial(initial_pos);
BR = bezier_curve_jump_initial(initial_pos);
BR(2, :) = -BR(2, :); % Reflect the bezier curve for the right leg

% Iterate over time steps
for i = 1:N % 22nd iteration t > p.t_start
    t = t_sim(i);
    if (t < p.t_start) % Delay 
        s = 0;
        s0 = 0;
    elseif (ge(t,p.t_first_pt) && (t <= p.t_second_pt))
        flag = flag + 1 ;
        % s = (t - p.t_first_pt) / p.gait_period_first;
        s = (mod(t - p.t_first_pt, p.gait_period_first)) / p.gait_period_first; % gait_period_first = 0.07
        if (s > s0)
        % if (s > s0 && flag == 1)
            % while(s <= 1)
                % Generate bezier curve 1 for jump trajectory
                [B_swing] = bezier_curve_jump_traj_1(initial_pos);
                BL = B_swing; % 3x5 matrix
                BR = B_swing;
                BR(2, :) = -BR (2, :);

                % plot for control point
                % plot(t,BR);
                % hold on;
            % end
        end
        s0 = s;
    elseif (ge(t, p.t_second_pt) && (t <= p.t_third_pt))
        flag = flag + 2 ;
        % s = (t - p.t_second_pt) / p.gait_period_second;
        s = (mod(t - p.t_second_pt, p.gait_period_second)) / p.gait_period_second; % gait_period_second = 0.03
        if (s > s0)
        % if (s < s0 && flag == 2)
            % while(s <= 1)
                % Generate bezier curve 2 for jump trajectory
                [B_swing] = bezier_curve_jump_traj_2(initial_pos, jump_height);
                BL = B_swing;
                BR = B_swing;
                BR(2, :) = -BR(2, :);
                % plot(t,BR);
                % hold on;
            % end
        end
        s0 = s;
        
    elseif (ge(t, p.t_third_pt) && (t <= p.t_fourth_pt))
        flag = flag + 3 ;
        % s = (t - p.t_third_pt) / p.gait_period_third;
        s = (mod(t - p.t_third_pt, p.gait_period_third)) / p.gait_period_third; % gait_period_third = 0.05
        if (s > s0)
        % if (s < s0 && flag == 3)
            % while(s <= 1)
                % Generate bezier curve 3 for jump trajectory
                [B_swing] = bezier_curve_jump_traj_3(initial_pos, jump_height);
                BL = B_swing;
                BR = B_swing;
                BR(2, :) = -BR(2, :);
                % plot(t,BR);
                % hold on;
            % end
        end  
        s0 = s;
    end
    
    % Compute leg positions using bezier curves
    % pos_foot_L_ref = 3x1 
    % BL = 3x5
    pos_foot_L_ref = bezier4(BL, s);
    pos_foot_R_ref = bezier4(BR, s);
    
    % Store leg positions
    q_ref_L(:, i) = pos_foot_L_ref;
    q_ref_R(:, i) = pos_foot_R_ref;
end

% plot for bezier curve
% figure
% % subplot()
% % plot(var.time,var.signals.values(:,4));
% plot(t_end, q_ref_R(:,4));
% grid on;
% % legend('');
% xlabel('time');
% ylabel('z-coordinate');
% title('z-coordinate for leg end position');

% Generate bezier curves for end stance
% BL = bezier_curve_jump_end(initial_pos);
% BR = bezier_curve_jump_end(initial_pos);
% BR(2, :) = -BR(2, :);

end

% Bezier curve evaluation function
function x = bezier4(B, s)
% Evaluate a 4th-order bezier curve

% x = zeros([3, 3]);
% Unable to perform assignment because the size
% of the left side is 3-by-1 and the size of the
% right side is 3-by-3.

x = zeros([3, 1]); % 3x1 matrix
for j = 0:4
    x = x + nchoosek(4, j) * (1 - s)^(4 - j) * s^j * B(:, j + 1);
end
end

%**************************************************************************
% Generate bezier curve parameters for jump trajectory
function [B_swing] = bezier_curve_jump_traj_1(initial_pos)
% Define bezier curve control points for a jump trajectory
k = initial_pos;
crouch_height = 0.8;
% Initial state 1st bezier curve
B_swing(:,1)  = [k(1); k(2); k(3)]; % zero velocity at initial point

% Bezier midpoint parameter
B_swing(:,2) = [k(1); k(2); k(3) + crouch_height]; % half way through crouching 

% End state of 1st bezier curve 
B_swing(:,3)  = [k(1); k(2); k(3) + crouch_height]; % full crouching

B_swing(:,4)  = [k(1); k(2); k(3) + crouch_height]; 
B_swing(:,5)  = [k(1); k(2); k(3) + crouch_height]; 

% B_swing(:,1)  = [k(1); k(2); k(3)]; % initial ascent
% B_swing(:,5)  = [k(1); k(2); k(3)]; % final touch down
% 
% % Bezier midpoint parameter
% B_swing(:,2) = [k(1); k(2); k(3) + crouch_height]; % crouching 
% B_swing(:,4) = [k(1); k(2); k(3) - jump_height/2]; % Landing approach
% 
% % landing 
% B_swing(:,3)  = [k(1); k(2); k(3) - jump_height]; % Highest point
end
%**************************************************************************
% Generate bezier curve parameters for jump trajectory
function [B_swing] = bezier_curve_jump_traj_2(initial_pos, jump_height)
% Define bezier curve control points for a jump trajectory
k = initial_pos;
crouch_height = 0.8;
% Initial state 2nd bezier curve
B_swing(:,1)  = [k(1); k(2); k(3) + crouch_height]; % end point of 1st curve

% Bezier midpoint parameter
B_swing(:,2) = [k(1); k(2); k(3) + crouch_height];
B_swing(:,3)  = [k(1); k(2); k(3) + crouch_height]; 

% End state of 2nd bezier curve 

B_swing(:,4) = [k(1); k(2); k(3)]; 
B_swing(:,5) = [k(1); k(2); k(3) - jump_height]; 
end
%**************************************************************************
% Generate bezier curve parameters for jump trajectory
function [B_swing] = bezier_curve_jump_traj_3(initial_pos, jump_height)
% Define bezier curve control points for a jump trajectory
k = initial_pos;
% Initial state 3rd bezier curve
B_swing(:,1) = [k(1); k(2); k(3) - jump_height]; 

B_swing(:,2) = [k(1); k(2); k(3) ]; 
% Bezier mdpoint parameter
B_swing(:,3)  = [k(1); k(2); k(3)]; 
B_swing(:,4)  = [k(1); k(2); k(3)]; 

% End state of 3rd bezier curve 
B_swing(:,5)  = [k(1); k(2); k(3)]; % zero velocity at end point
end
%**************************************************************************
% Generate initial bezier curve parameters
function [B_swing] = bezier_curve_jump_initial(initial_pos)
k = initial_pos;

B_swing(:, 1) = [k(1); k(2); k(3)];
B_swing(:, 5) = [k(1); k(2); k(3)];
B_swing(:, 2) = [k(1); k(2); k(3)];
B_swing(:, 4) = [k(1); k(2); k(3)];
B_swing(:, 3) = [k(1); k(2); k(3)];
end
%**************************************************************************
% Generate end bezier curve parameters
% function [B_swing] = bezier_curve_jump_end(initial_pos)
% k = initial_pos;
% 
% B_swing(:, 1) = [k(1); k(2); k(3)];
% B_swing(:, 5) = [k(1); k(2); k(3)];
% B_swing(:, 2) = [k(1); k(2); k(3)];
% B_swing(:, 4) = [k(1); k(2); k(3)];
% B_swing(:, 3) = [k(1); k(2); k(3)];
% end

%
