classdef APF_GUI_3< handle
    properties
        % GUI components
        fig
        ax_env
        ax_potential
        btn_run_apf
        btn_clear
        btn_add_obstacle
        btn_remove_obstacle
        btn_set_start
        btn_set_goal
        btn_show_3d_potential
        slider_zeta
        slider_eta
        slider_rho0
        slider_stepsize
        slider_repulsive_power
        slider_attractive_power
        slider_random_walk_strength
        checkbox_show_potential
        checkbox_random_walk
        
        % Environment data
        start_pos
        goal_pos
        obstacles % [x, y, radius]
        workspace = [0 100 0 100]
        
        % Algorithm parameters
        params = struct(...
            'zeta', 1.0, ...          % Attractive gain
            'eta', 100.0, ...          % Repulsive gain
            'rho_0', 20.0, ...         % Obstacle influence distance
            'step_size', 1.0, ...      % Movement step size
            'goal_threshold', 2.0, ... % Distance to consider goal reached
            'epsilon', 0.1, ...       % Minimum force threshold
            'max_iter', 1000, ...      % Maximum iterations
            'repulsive_power', 2, ...  % Power of repulsive potential
            'attractive_power', 1, ... % Power of attractive potential
            'random_walk_strength', 0.5, ... % Strength of random walk
            'use_random_walk', false, ... % Enable random walk
            'd_star_goal', 30)        % Threshold distance for attractive field
        
        % Results
        apf_path
        computation_time
        potential_grid % Stores the computed potential field grid
    end
    
    methods
        function obj = APF_GUI_3()
            % Create main figure
            obj.fig = figure('Name', 'APF Path Planner', 'NumberTitle', 'off', ...
                'Position', [100, 80, 1200, 600], 'Resize', 'off');
            
            % Create environment axes
            obj.ax_env = subplot(1, 3, 2);
            axis(obj.ax_env, obj.workspace);
            grid(obj.ax_env, 'on');
            title(obj.ax_env, 'Environment');
            hold(obj.ax_env, 'on');
            
            % Create potential field axes
            obj.ax_potential = subplot(1, 3, 3);
            title(obj.ax_potential, 'Potential Field');
            hold(obj.ax_potential, 'on');
            
            % Create buttons and controls
            obj.create_controls();
            
            % Initialize with default start and goal
            obj.start_pos = [20, 20];
            obj.goal_pos = [80, 80];
            
            % Initialize with obstacles
            obj.obstacles = [
                30, 40, 8;    % Large center-left obstacle
                70, 60, 7;    % Large upper-right obstacle
                50, 20, 6;    % Medium lower-center obstacle
                20, 70, 4;    % Small upper-left
                80, 30, 4;    % Small lower-right
                40, 50, 3;    % Small center
                60, 40, 3;    % Small center-right
                10, 50, 2;    % Left boundary
                90, 50, 2;    % Right boundary
                50, 10, 2;    % Bottom boundary
                50, 90, 2     % Top boundary
            ];
            
            % Update display
            obj.update_display();
        end
        
        function create_controls(obj)
            % Panel for controls
            control_panel = uipanel('Title', 'Controls', 'Position', [0.02, 0.02, 0.25, 0.96]);
            
            % Buttons
            obj.btn_run_apf = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                'String', 'Run APF', 'Position', [10, 500, 100, 30], ...
                'Callback', @(~,~) obj.run_apf());
            
            obj.btn_clear = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                'String', 'Clear Path', 'Position', [10, 460, 100, 30], ...
                'Callback', @(~,~) obj.clear_path());
            
            obj.btn_add_obstacle = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                'String', 'Add Obstacle', 'Position', [120, 460, 100, 30], ...
                 'Callback', @(~,~) obj.add_obstacle());
            obj.btn_remove_obstacle = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                'String', 'Remove Obstacle', 'Position', [120, 500, 100, 30], ...
                'Callback', @(~,~) obj.remove_obstacle());

            
            obj.btn_set_start = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                'String', 'Set Start', 'Position', [10, 420, 100, 30], ...
                'Callback', @(~,~) obj.set_start());
            
            obj.btn_set_goal = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                'String', 'Set Goal', 'Position', [120, 420, 100, 30], ...
                'Callback', @(~,~) obj.set_goal());
            
            obj.btn_show_3d_potential = uicontrol('Parent', control_panel, 'Style', 'pushbutton', ...
                'String', 'Show 3D Potential', 'Position', [10, 380, 210, 30], ...
                'Callback', @(~,~) obj.show_3d_potential());
            
            % Sliders for parameters
            obj.slider_zeta = obj.create_slider(control_panel, 340, 'Attractive Gain (zeta)', 0.1, 5, obj.params.zeta);
            obj.slider_eta = obj.create_slider(control_panel, 300, 'Repulsive Gain (eta)', 10, 500, obj.params.eta);
            obj.slider_rho0 = obj.create_slider(control_panel, 260, 'Obstacle Influence (rho0)', 5, 50, obj.params.rho_0);
            obj.slider_stepsize = obj.create_slider(control_panel, 220, 'Step Size', 0.1, 5, obj.params.step_size);
            obj.slider_repulsive_power = obj.create_slider(control_panel, 180, 'Repulsive Power', 1, 4, obj.params.repulsive_power);
            obj.slider_attractive_power = obj.create_slider(control_panel, 140, 'Attractive Power', 1, 3, obj.params.attractive_power);
            obj.slider_random_walk_strength = obj.create_slider(control_panel, 100, 'Random Walk Strength', 0, 2, obj.params.random_walk_strength);
            
            % Checkboxes
            obj.checkbox_show_potential = uicontrol('Parent', control_panel, 'Style', 'checkbox', ...
                'String', 'Show Potential Field', 'Position', [10, 70, 150, 30], ...
                'Value', 0, 'Callback', @(~,~) obj.update_display());
            
            obj.checkbox_random_walk = uicontrol('Parent', control_panel, 'Style', 'checkbox', ...
                'String', 'Enable Random Walk', 'Position', [10, 40, 150, 30], ...
                'Value', obj.params.use_random_walk, ...
                'Callback', @(~,~) obj.toggle_random_walk());
        end
        
        function show_3d_potential(obj)
            [X, Y] = meshgrid(linspace(obj.workspace(1), obj.workspace(2), 100), ...
                             linspace(obj.workspace(3), obj.workspace(4), 100));
            
            U_total = zeros(size(X));
            U_att = zeros(size(X));
            U_rep = zeros(size(X));
            
            for i = 1:numel(X)
                pos = [X(i), Y(i)];
                [U_att(i), ~] = obj.attractive_potential(pos);
                [U_rep(i), ~] = obj.repulsive_potential(pos);
                U_total(i) = U_att(i) + U_rep(i);
            end
            
            fig_3d = figure('Name', '3D Potential Field', 'Position', [200, 80, 1000, 700]);
            
            subplot(2,2,1);
            surf(X, Y, U_total, 'EdgeColor', 'none');
            title('Total Potential Field');
            xlabel('X'); ylabel('Y'); zlabel('Potential');
            colorbar;
            view(3);
            
            subplot(2,2,2);
            surf(X, Y, U_att, 'EdgeColor', 'none');
            title('Attractive Potential');
            xlabel('X'); ylabel('Y'); zlabel('Potential');
            colorbar;
            view(3);
            
            subplot(2,2,3);
            surf(X, Y, U_rep, 'EdgeColor', 'none');
            title('Repulsive Potential');
            xlabel('X'); ylabel('Y'); zlabel('Potential');
            colorbar;
            view(3);
            
            subplot(2,2,4);
            contourf(X, Y, U_total, 20);
            title('Potential Field Contour');
            xlabel('X'); ylabel('Y');
            colorbar;
            hold on;
            plot(obj.start_pos(1), obj.start_pos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
            plot(obj.goal_pos(1), obj.goal_pos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            
            for i = 1:size(obj.obstacles, 1)
                rectangle('Position', [obj.obstacles(i,1)-obj.obstacles(i,3), ...
                                      obj.obstacles(i,2)-obj.obstacles(i,3), ...
                                      2*obj.obstacles(i,3), 2*obj.obstacles(i,3)], ...
                         'Curvature', [1 1], 'FaceColor', [0.5 0.5 0.5]);
            end
            
            if ~isempty(obj.apf_path)
                plot(obj.apf_path(:,1), obj.apf_path(:,2), 'b-', 'LineWidth', 2);
            end
            
            axis equal;
            grid on;
            hold off;
        end
        
        function slider = create_slider(obj, parent, ypos, label, minval, maxval, initval)
            uicontrol('Parent', parent, 'Style', 'text', ...
                'String', label, 'Position', [10, ypos+20, 180, 20]);
            
            slider = uicontrol('Parent', parent, 'Style', 'slider', ...
                'Min', minval, 'Max', maxval, 'Value', initval, ...
                'Position', [10, ypos, 180, 20], ...
                'Callback', @(~,~) obj.update_parameters());
            
            uicontrol('Parent', parent, 'Style', 'text', ...
                'String', num2str(initval), 'Position', [200, ypos, 50, 20], ...
                'Tag', [label '_value']);
        end
        
        function update_parameters(obj)
            obj.params.zeta = obj.slider_zeta.Value;
            obj.params.eta = obj.slider_eta.Value;
            obj.params.rho_0 = obj.slider_rho0.Value;
            obj.params.step_size = obj.slider_stepsize.Value;
            obj.params.repulsive_power = obj.slider_repulsive_power.Value;
            obj.params.attractive_power = obj.slider_attractive_power.Value;
            obj.params.random_walk_strength = obj.slider_random_walk_strength.Value;
            
            % Update displayed values
            set(findobj(obj.fig, 'Tag', 'Attractive Gain (zeta)_value'), 'String', num2str(obj.params.zeta, '%.2f'));
            set(findobj(obj.fig, 'Tag', 'Repulsive Gain (eta)_value'), 'String', num2str(obj.params.eta, '%.1f'));
            set(findobj(obj.fig, 'Tag', 'Obstacle Influence (rho0)_value'), 'String', num2str(obj.params.rho_0, '%.1f'));
            set(findobj(obj.fig, 'Tag', 'Step Size_value'), 'String', num2str(obj.params.step_size, '%.2f'));
            set(findobj(obj.fig, 'Tag', 'Repulsive Power_value'), 'String', num2str(obj.params.repulsive_power, '%.1f'));
            set(findobj(obj.fig, 'Tag', 'Attractive Power_value'), 'String', num2str(obj.params.attractive_power, '%.1f'));
            set(findobj(obj.fig, 'Tag', 'Random Walk Strength_value'), 'String', num2str(obj.params.random_walk_strength, '%.1f'));
            
            obj.update_display();
        end
        
        function toggle_random_walk(obj)
            obj.params.use_random_walk = obj.checkbox_random_walk.Value;
        end
        
        function update_display(obj)
            cla(obj.ax_env);
            cla(obj.ax_potential);
            
            obj.plot_environment(obj.ax_env);
            
            if obj.checkbox_show_potential.Value
                obj.plot_potential_field();
            end
            
            % Store handles for legend
            legend_handles = [];
            legend_labels = {};
            
            % Plot start and goal
            h_start = plot(obj.ax_env, obj.start_pos(1), obj.start_pos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
            h_goal = plot(obj.ax_env, obj.goal_pos(1), obj.goal_pos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            legend_handles = [legend_handles, h_start, h_goal];
            legend_labels = [legend_labels, {'Start', 'Goal'}];
            
            % Create a dummy obstacle handle for legend
            if ~isempty(obj.obstacles)
                h_obstacle = plot(obj.ax_env, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerEdgeColor', [0.5 0.5 0.5]);
                legend_handles = [legend_handles, h_obstacle];
                legend_labels = [legend_labels, {'Obstacles'}];
            end
            
            % Plot APF path
            if ~isempty(obj.apf_path)
                h_apf = plot(obj.ax_env, obj.apf_path(:,1), obj.apf_path(:,2), 'b-', 'LineWidth', 2);
                plot(obj.ax_env, obj.apf_path(:,1), obj.apf_path(:,2), 'b.', 'MarkerSize', 10);
                legend_handles = [legend_handles, h_apf];
                legend_labels = [legend_labels, {'APF Path'}];
            end
            
            % Create legend
            if ~isempty(legend_handles)
                legend(obj.ax_env, legend_handles, legend_labels, 'Location', 'best');
            end
        end
        
        function plot_environment(obj, ax)
            plot(ax, obj.start_pos(1), obj.start_pos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
            plot(ax, obj.goal_pos(1), obj.goal_pos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            
            for i = 1:size(obj.obstacles, 1)
                rectangle(ax, 'Position', [obj.obstacles(i,1)-obj.obstacles(i,3), ...
                    obj.obstacles(i,2)-obj.obstacles(i,3), ...
                    2*obj.obstacles(i,3), 2*obj.obstacles(i,3)], ...
                    'Curvature', [1 1], 'FaceColor', [0.5 0.5 0.5]);
            end
            
            axis(ax, 'equal');
            grid(ax, 'on');
            xlim(ax, [obj.workspace(1), obj.workspace(2)]);
            ylim(ax, [obj.workspace(3), obj.workspace(4)]);
            title(ax, 'Path Planning Environment');
        end
        
        function plot_potential_field(obj)
            % Create grid for potential field calculation
            [X, Y] = meshgrid(linspace(obj.workspace(1), obj.workspace(2), 50), ...
                             linspace(obj.workspace(3), obj.workspace(4), 50));
            
            U_total = zeros(size(X));
            
            % Calculate potential at each grid point
            for i = 1:numel(X)
                pos = [X(i), Y(i)];
                [U_att, ~] = obj.attractive_potential(pos);
                [U_rep, ~] = obj.repulsive_potential(pos);
                U_total(i) = U_att + U_rep;
            end
            
            % Store the potential grid
            obj.potential_grid = U_total;
            
            % Plot the potential field
            surf(obj.ax_potential, X, Y, U_total);
            title(obj.ax_potential, 'Total Potential Field');
            xlabel(obj.ax_potential, 'X');
            ylabel(obj.ax_potential, 'Y');
            zlabel(obj.ax_potential, 'Potential');
            shading(obj.ax_potential, 'interp');
            colorbar(obj.ax_potential);
            
            % Calculate and plot gradient vectors
            [FX, FY] = gradient(U_total);
            
            % Downsample for clearer visualization
            step = 5;
            quiver(obj.ax_potential, X(1:step:end, 1:step:end), Y(1:step:end, 1:step:end), ...
                   -FX(1:step:end, 1:step:end), -FY(1:step:end, 1:step:end), 'r');
        end
        
        function [U_att, F_att] = attractive_potential(obj, pos)
            d = norm(pos - obj.goal_pos);
            
            if d <= obj.params.d_star_goal
                U_att = 0.5 * obj.params.zeta * d^2;
                if d > 0
                    F_att = -obj.params.zeta * (pos - obj.goal_pos);
                else
                    F_att = [0, 0];
                end
            else
                U_att = obj.params.d_star_goal * obj.params.zeta * d - 0.5 * obj.params.zeta * obj.params.d_star_goal^2;
                if d > 0
                    F_att = -obj.params.d_star_goal * obj.params.zeta * (pos - obj.goal_pos)/d;
                else
                    F_att = [0, 0];
                end
            end
            
            % Apply attractive power parameter
            if obj.params.attractive_power ~= 1
                U_att = U_att * (d^obj.params.attractive_power) / (d^2);
                F_att = F_att * (d^obj.params.attractive_power) / (d^2);
            end
        end
        
        function [U_rep, F_rep] = repulsive_potential(obj, pos)
            U_rep = 0;
            F_rep = [0, 0];
            
            for i = 1:size(obj.obstacles, 1)
                obstacle_pos = obj.obstacles(i, 1:2);
                obstacle_radius = obj.obstacles(i, 3);
                dist_to_obstacle = norm(pos - obstacle_pos) - obstacle_radius;
                
                if dist_to_obstacle <= obj.params.rho_0
                    if dist_to_obstacle <= 0.1, dist_to_obstacle = 0.1; end
                    
                    U_rep = U_rep + 0.5 * obj.params.eta * ...
                           (1/dist_to_obstacle - 1/obj.params.rho_0)^obj.params.repulsive_power;
                    
                    if dist_to_obstacle > 0
                        F_rep = F_rep + obj.params.eta * ...
                               (1/dist_to_obstacle - 1/obj.params.rho_0)^(obj.params.repulsive_power-1) * ...
                               (1/dist_to_obstacle^2) * (pos - obstacle_pos)/norm(pos - obstacle_pos);
                    end
                end
            end
        end
        
        function run_apf(obj)
            tic;
            
            obj.apf_path = obj.start_pos;
            current_pos = obj.start_pos;
            stuck_counter = 0;
            
            for iter = 1:obj.params.max_iter
                if norm(current_pos - obj.goal_pos) < obj.params.goal_threshold
                    disp('APF: Goal reached!');
                    break;
                end
                
                % Calculate forces using gradient of potential field
                if obj.checkbox_show_potential.Value && ~isempty(obj.potential_grid)
                    % Use precomputed potential grid gradient
                    [X, Y] = meshgrid(linspace(obj.workspace(1), obj.workspace(2), 50), ...
                                     linspace(obj.workspace(3), obj.workspace(4), 50));
                    
                    % Find closest grid point
                    [~, idx_x] = min(abs(X(1,:) - current_pos(1)));
                    [~, idx_y] = min(abs(Y(:,1) - current_pos(2)));
                    
                    % Calculate gradient at this point
                    [FX, FY] = gradient(obj.potential_grid);
                    F_total = -[FX(idx_y, idx_x), FY(idx_y, idx_x)];
                    
                    % Normalize and scale by step size
                    if norm(F_total) > 0
                        F_total = F_total/norm(F_total) * obj.params.step_size;
                    end
                else
                    % Use analytical gradients
                    [~, F_att] = obj.attractive_potential(current_pos);
                    [~, F_rep] = obj.repulsive_potential(current_pos);
                    F_total = F_att + F_rep;
                    
                    % Normalize and scale by step size
                    if norm(F_total) > 0
                        F_total = F_total/norm(F_total) * obj.params.step_size;
                    end
                end
                
                % Handle local minima with random walk
                if iter > 10 && norm(F_total) < obj.params.epsilon
                    stuck_counter = stuck_counter + 1;
                    
                    if obj.params.use_random_walk && stuck_counter > 5
                        F_total = F_total + randn(1,2) * obj.params.step_size * obj.params.random_walk_strength;
                        disp('APF: Applying random walk');
                        stuck_counter = 0;
                    else
                        disp('APF: Stuck in local minimum');
                        break;
                    end
                else
                    stuck_counter = 0;
                end
                
                % Update position
                new_pos = current_pos + F_total;
                new_pos = max(new_pos, [obj.workspace(1), obj.workspace(3)]);
                new_pos = min(new_pos, [obj.workspace(2), obj.workspace(4)]);
                
                obj.apf_path = [obj.apf_path; new_pos];
                current_pos = new_pos;
            end
            
            obj.computation_time = toc;
            disp(['APF computation time: ', num2str(obj.computation_time), ' seconds']);
            
            path_length = sum(sqrt(sum(diff(obj.apf_path).^2, 2)));
            disp(['APF path length: ', num2str(path_length)]);
            
            obj.update_display();
        end
        
        function clear_path(obj)
            obj.apf_path = [];
            obj.update_display();
        end
        
        function add_obstacle(obj)
            disp('Click on the environment to add an obstacle center');
            [x, y] = ginput(1);
            
            disp('Enter obstacle radius:');
            radius = inputdlg('Obstacle radius:', 'Add Obstacle', 1, {'6'});
            radius = str2double(radius{1});
            
            obj.obstacles = [obj.obstacles; x, y, radius];
            obj.update_display();
        end
        function remove_obstacle(obj)
        disp('Click near the obstacle you want to remove');
        [x, y] = ginput(1);
        click_pos = [x, y];
    
        if isempty(obj.obstacles)
        disp('No obstacles to remove.');
        return;
        end

        % Find closest obstacle
        distances = vecnorm(obj.obstacles(:, 1:2) - click_pos, 2, 2) - obj.obstacles(:,3);
        [min_dist, idx] = min(distances);

        % Define a threshold to consider as "clicked"
        if min_dist < 5  % You can tune this threshold
        obj.obstacles(idx, :) = [];
        disp('Obstacle removed.');
        else
            disp('No obstacle found near the clicked point.');
        end

        obj.update_display();
end

        
        function set_start(obj)
            disp('Click on the environment to set start position');
            [x, y] = ginput(1);
            obj.start_pos = [x, y];
            obj.update_display();
        end
        
        function set_goal(obj)
            disp('Click on the environment to set goal position');
            [x, y] = ginput(1);
            obj.goal_pos = [x, y];
            obj.update_display();
        end
    end
end