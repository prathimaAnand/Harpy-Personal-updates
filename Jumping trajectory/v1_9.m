function [q_ref_L, q_ref_R, t_sim,s0] = bezier_curve_jump(initial_pos, jump_height)
% This function generates bezier curve trajectories for leg motion during jumping.

% Things to work on:
% 15 point bezier - 3 curve stitching - Done
% placing of control points - Done
% % % decrease s value 
% instantaneous jump - 
% smooth transition from stance to crouch
% add end stance bezier
  
p.gait_period = 0.4;          % Period of the gait cycle
p.t_start = 0.2;              % Start time
gait_length = 3;              % Number of gait cycles
p.t_end = gait_length * p.gait_period + p.t_start; % End time of the simulation % t_end = 0.9
p.dt = 0.005;                 % Time step
% new tim pararmeters
% p.t_end = p.t_start + gait_length * p.gait_period;
% p.dt = 0.001;      
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
    else
        % s = mod((t - p.t_start, p.gait_period)/p.gait_period, 1);
        % s = mod(((t - p.t_start, p.gait_period)/(p.gait_period)), 1.0125) ; % difference s & its previous = 0.0125
        s = mod((t - p.t_start)/p.gait_period, 1); % difference s & its previous = 0.0125
        if(s0(i-1) > s)
            flag = flag + 1;
        end
        % if(s0(1,i-1) > s && s0(1,i-1) < 1)
        %     s0(1,i-1) = 1; 
        %     flag = flag + 1;
        %     if(s0(1,i-1) > 1)
        %         s0(1,i-1) = 1; 
        %     elseif(s0(1,i-1) < 0)
        %         s0(1,i-1) = 0;
        %     end
        % end
        % Generate bezier curve 1 for jump trajectory
        if(flag == 0)
            [B_swing] = bezier_curve_jump_traj_1(initial_pos);
            BL = B_swing; % 3x5 matrix
            BR = B_swing;
            BR(2, :) = -BR (2, :);
        
        elseif(flag == 1)
            [B_swing] = bezier_curve_jump_traj_2(initial_pos, jump_height);
            p.gait_period = 0.2;
            BL = B_swing; % 3x5 matrix
            BR = B_swing;
            BR(2, :) = -BR (2, :);
        % p.gait_period = 0.4;

        elseif(flag == 2)
            [B_swing] = bezier_curve_jump_traj_3(initial_pos, jump_height);
            BL = B_swing; % 3x5 matrix
            BR = B_swing;
            BR(2, :) = -BR (2, :);
        
        elseif(flag == 3)
            [B_swing] = bezier_curve_jump_traj_1(initial_pos);
            BL = B_swing; % 3x5 matrix
            BR = B_swing;
            BR(2, :) = -BR (2, :);
        
        elseif(flag == 4)
            [B_swing] = bezier_curve_jump_traj_1(initial_pos);
            BL = B_swing; % 3x5 matrix
            BR = B_swing;
            BR(2, :) = -BR (2, :);
        
        elseif(flag == 5)
            [B_swing] = bezier_curve_jump_traj_1(initial_pos);
            BL = B_swing; % 3x5 matrix
            BR = B_swing;
            BR(2, :) = -BR (2, :);
        end
        % plot for control point
        % plot(t,BR);
        % hold on;
        % s0 = s;
    end
    s0(i) = s;
    % Compute leg positions using bezier curves : pos_foot_L_ref = 3x1, BL = 3x5

    % Store leg positions
    if( flag == 1)
        pos_foot_L_ref = bezier4(BL, 2*s);
        pos_foot_R_ref = bezier4(BR, 2*s);
    else
        pos_foot_L_ref = bezier4(BL, s);
        pos_foot_R_ref = bezier4(BR, s);
    end
    q_ref_L(:, i) = pos_foot_L_ref;
    q_ref_R(:, i) = pos_foot_R_ref;
end
figure
% plot s value
plot(t_sim(1,:), s0(1,:));
xlabel('time');
ylabel('s value');
title('s value');

figure
% plotting leg end position
plot(transpose(t_sim(1,:)),(q_ref_L(3,:)));
xlabel('time ');
ylabel('z-axis');
title('z-axis of Leg end position');

% Generate bezier curves for end stance
% BL = bezier_curve_jump_end(initial_pos);
% BR = bezier_curve_jump_end(initial_pos);
% BR(2, :) = -BR(2, :);
end

% Bezier curve evaluation function
function x = bezier4(B, s)
% Evaluate a 4th-order bezier curve

% if (s  > 1 )
%     s = 1;
% end
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
B_swing(:,2) = [k(1); k(2); k(3)]; % half way through crouching 
B_swing(:,3)  = [k(1); k(2); k(3) + crouch_height * (1/2)]; % full crouching
% End state of 1st bezier curve 
B_swing(:,4)  = [k(1); k(2); k(3) + crouch_height]; 
B_swing(:,5)  = [k(1); k(2); k(3) + crouch_height]; 

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
B_swing(:,3)  = [k(1); k(2); k(3) ]; 
% End state of 2nd bezier curve 
B_swing(:,4) = [k(1); k(2); k(3) - jump_height]; 
B_swing(:,5) = [k(1); k(2); k(3) - jump_height]; 
end
%**************************************************************************
% Generate bezier curve parameters for jump trajectory
function [B_swing] = bezier_curve_jump_traj_3(initial_pos, jump_height)
% Define bezier curve control points for a jump trajectory
k = initial_pos;
% Initial state 3rd bezier curve
B_swing(:,1) = [k(1); k(2); k(3) - jump_height]; 
% Bezier mdpoint parameter
B_swing(:,2) = [k(1); k(2); k(3) - jump_height]; 
B_swing(:,3)  = [k(1); k(2); k(3) - jump_height*(1/2)]; 
B_swing(:,4)  = [k(1); k(2); k(3)]; 
% End state of 3rd bezier curve 
B_swing(:,5)  = [k(1); k(2); k(3)]; % zero velocity at end point
end
%**************************************************************************
% Generate initial bezier curve parameters
function [B_swing] = bezier_curve_jump_initial(initial_pos)
k = initial_pos;

B_swing(:, 1) = [k(1); k(2); k(3)];
B_swing(:, 2) = [k(1); k(2); k(3)];
B_swing(:, 3) = [k(1); k(2); k(3)];
B_swing(:, 4) = [k(1); k(2); k(3)];
B_swing(:, 5) = [k(1); k(2); k(3)];
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
