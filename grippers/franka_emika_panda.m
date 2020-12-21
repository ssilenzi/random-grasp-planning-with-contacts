classdef franka_emika_panda < matlab.mixin.Copyable
    properties (Access = public)
        rob_model;          % Robot model from Robotic System Toolbox
        n_dof;              % no. of degrees of freedom
        n_links;            % no. of links
        n_ees;              % no. of end-effectors
        n_contacts;         % no. of max finger contacts
        frame_names;        % array with names of all link frames
        ee_names;           % array with names of ee (left, right, wrist)
        home_config;        % home joint configuration of the robot
        q;                  % present joint configuration
        X;                  % FK in hom. mats. (left, right, wrist)
        T_all;              % FK in hom. mats. to all frames
        J;                  % Jacobian to the fingers left and right
        J_w;                % Jacobian to the wrist
    end
    methods
        % Constructor
        function obj = franka_emika_panda()
            % Loading the robot using Robotic System Toolbox
            obj.rob_model = loadrobot('frankaEmikaPanda');
            
            % Setting constant parameters
            obj.n_dof = numel(homeConfiguration(obj.rob_model));
            obj.n_links = obj.n_dof; % not considering panda_link0 (base)
            obj.n_ees = 3; % left, right, wrist
            obj.n_contacts = 2;
            obj.frame_names = {};
            for k = 1:obj.n_links
                body_k = obj.rob_model.Bodies{k};
                obj.frame_names{k} = body_k.Name;
            end
            obj.ee_names = {'panda_leftfinger', 'panda_rightfinger', 'panda_hand'};
            obj.home_config = homeConfiguration(obj.rob_model);
            
            % Initializing the variables
            obj.q = obj.home_config;
            obj.X = zeros(4,4,3);
            obj.T_all = zeros(4,4,obj.n_links);
            obj.J = [zeros(6, obj.n_dof);
                zeros(6, obj.n_dof)];
            obj.J_w = zeros(6, obj.n_dof);

            % Setting the configuration and updating every variable
            set_config(obj, obj.q);
        end
        
        % Some auxiliary get functions
        function n = get_n_dof(obj)
            n = obj.n_dof;
        end
        function n = get_n_contacts(obj)
            n = obj.n_contacts;
        end
        function T = get_T_all(obj)
            T = obj.T_all;
        end
        function X = get_forward_kinematics(obj)
            X = obj.X;
        end
        function J = get_jacobian(obj)
            J = obj.J;
        end
        function Jw = get_wrist_jacobian(obj)
            Jw = obj.J_wrist;
        end
        function [Jp_l, Jp_r, Jp_w] = get_pos_jacobians(obj)
            J_tmp = obj.J;
            Jp_l = J_tmp(1:3,:);
            Jp_r = J_tmp(7:9,:);
            Jp_w = obj.J_wrist(1:3,:);
        end
                      
        % Set configuration function
        function set_config(obj, q)
            % q is a vector of 11 doubles (checking dimensions)
            if(length(q) ~= obj.n_dof)
                fprintf(['Not correct configuration vector. ', ...
                    'It must be dimension %d\n'], obj.n_dof);
                return;
            end
            obj.q = q;
            obj.compute_forward_kinematics();
            obj.compute_jacobian();
            obj.compute_wrist_jacobian();
        end
        
        % Function for plotting the robot using show
        function hr = plot(obj)
% %             hold on;
            hr = show(obj.rob_model);
            xlabel('z');
            ylabel('x');
            zlabel('y');
        end
        
        % Forward Kinematics function (to all frames and to the three ees)
        function compute_forward_kinematics(obj)            
            % Setting all the frames fk
            for k = 1:obj.n_links
                obj.T_all(:,:,k) = ...
                    getTransform(obj.rob_model, obj.q, obj.frame_names{k});
            end
            
            % Setting the fk for the ees
            for k = 1:obj.n_ees
                obj.X(:,:,k) = ...
                    getTransform(obj.rob_model, obj.q, obj.ee_names{k});
            end            
        end
        
        % Function for computing the geometric jacobian of fingers
        function compute_jacobian(obj)
            % Getting the two jacobians
            % In RSToolbox on top orientation and below position
            jac_left = ...
                geometricJacobian(obj.rob_model, obj.q, obj.ee_names{1});
            jac_right = ...
                geometricJacobian(obj.rob_model, obj.q, obj.ee_names{2});
            
            % Flipping and putting position above
            tmp_jac = jac_left(1:3,:);          % left
            jac_left(1:3,:) = jac_left(4:6,:);
            jac_left(4:6,:) = tmp_jac;
            
            tmp_jac = jac_right(1:3,:);          % right
            jac_right(1:3,:) = jac_right(4:6,:);
            jac_right(4:6,:) = tmp_jac;
            
            % Putting together inside the class
            % ATT!!! Not sure if it is better to separate position parts
            % from orientation parts and put them together
            obj.J = [jac_left; jac_right]; 
            
        end
        
        % Function for computing the geometric jacobian wrist
        function compute_wrist_jacobian(obj)
            % Getting the wrist jacobian
            % In RSToolbox on top orientation and below position
            jac_wrist = ...
                geometricJacobian(obj.rob_model, obj.q, obj.ee_names{3});
            
            % Flipping and putting position above
            tmp_jac = jac_wrist(1:3,:);          % wrist
            jac_wrist(1:3,:) = jac_wrist(4:6,:);
            jac_wrist(4:6,:) = tmp_jac;

            % Putting together inside the class
            obj.J_w = jac_wrist;
        end
        
        % Inverse Kinematics function for simple frame positioning
        function [q_sol, ne] = compute_simple_inverse_kinematics(obj, Xd, ...
                frame_name, weights, rand_restart)
            
            % This inverse kinematics is based on the default RSToolbox
            % Given the desired pose and the frame name, computes the
            % joints needed to reach that pose with the link
            
            % Checking if the frame exists and saving index
            found = false;
            ind = [];
            for k = 1:length(obj.frame_names)
                if strcmp(frame_name, obj.frame_names{k})
                    found = true;
                    ind = k;
                end
            end
            if ~found
                disp('The requested frame is '); disp(frame_name);
                error('Cannot do ik: the requested frame does not exist!');
            end
            
            if ~exist('weights', 'var')
                weights = [1 1 1 1 1 1]; % Weights of the ik algorithm
            end
            if ~exist('rand_restart', 'var')
                % If bad sol. found, restart with random guess
                rand_restart = false; 
            end
            
            % Setting params
            ik = inverseKinematics('RigidBodyTree', obj.rob_model);
            ik.SolverParameters.AllowRandomRestart = rand_restart;
            
            % Solving ik and setting robot
            q_sol = ik(frame_name, Xd, weights, obj.q);
            obj.set_config(q_sol);
            
            % Computing error
            error_now = Xd - obj.T_all(:,:,ind);
            ne = norm(error_now);
        end

        % Inverse Kinematics function using Stack of Tasks
        function ne = compute_differential_inverse_kinematics(obj, ...
                Xd, q_open_d, enable_contact, integration_step, ...
                try_max , tol, pinvtol)
            
            % This differential IK is priority based inversion for the
            % three desired poses of finger-tips and wrist.
            % As of now, higher priorities given to fingers and least to
            % the wrist.
            % [Siciliano Slotine, A General Framework for managing ...]
            % This function also works for unilateral constraints on the
            % gripper joint:
            % [Mansard, A unified approach to integrate unilateral...]
            % HOW TO USE:
            % If q_open_d has only two elements (gripper joints and not
            % remaining joints) -> Unilateral constraints
            % If q_open_d has eight elements (also other robot joints) 
            % -> Nearest solution to q_open_d
            
            if ~exist('try_max', 'var')
                try_max = 100; % Max of iterations allowed in diff inv kin
            end
            if ~exist('enable_contact', 'var')
                enable_contact = [1;1];
                % Max of iterations allowed in diff inv kin
            end
            if ~exist('integration_step', 'var')
                integration_step = 1/10; % integration step for diff inv kin
            end
            if ~exist('tol', 'var')
                tol = .01; % Tolerance to define if a target is reached
            end
            if ~exist('pinvtol', 'var')
                pinvtol = .01; % Tolerance for "cut-off" pinv
            end
            if ~exist('lam_unil', 'var')
                lam_unil = 1; % Velocity for unilateral constraints
            end
            
            % Checking if there is also a wrist reference
            is_wrist = false;
            if(isequal([4 4 3], size(Xd)))
                is_wrist = true;
            end
            
            % Unilateral or minimum difference?
            if size(q_open_d,1) == obj.n_contacts
                is_unil = true;
            elseif size(q_open_d,1) == obj.n_dof
                is_unil = false;
            else
                error('Do not know what to do! In IK, unexpected joints size!');
            end
            
            ntry = 1;  ne = inf;
            while ntry < try_max && ne > tol
                
                % Compute the position error and the jacobians 
                % for tasks 1 and 2
                e1 = (Xd(1:3,4,1) - obj.X(1:3,4,1))*enable_contact(1); % left finger
                e2 = (Xd(1:3,4,2) - obj.X(1:3,4,2))*enable_contact(2); % right finger
                [J1, J2, ~] = obj.get_pos_jacobians();
                
                % Now computing task 3 according to the flags
                % If there is a wrist ref. then get a task 4
                if is_wrist
                    e3 = (Xd(1:3,4,3) - obj.X(1:3,4,3));
                    [~, ~, J3] = obj.get_pos_jacobians();
                else
                    e3 = zeros(3,1);
                    J3 = zeros(3,size(obj.q,1));
                end
                
                if is_unil  % For the unilateral constraint
                    % Compute error, H matrix and jacobian
                    e4 = q_open_d - obj.q(7:8);
                    lvec = e4 <= 0;
                    H78 = [lvec(1), 0; 0, lvec(2)];
                    H = [zeros(2,6), H78];                    
                    J4 = lam_unil * H;
                else        % For minumum difference from q_open_d
                    % error and jacobian for keeping configuration
                    e4 = q_open_d - obj.q;
                    J4 = eye(size(obj.q,1));
                end
                                                              
                % Computing pseudo-invs and projectors
                pJ1 = pinv(J1,pinvtol);
                P1 = (eye(size(obj.sig_act,1)) - pJ1*J1);
                P12 = P1 - pinv(J2*P1,pinvtol)*J2*P1;
                P123 = P12 - pinv(J3*P12,pinvtol)*J3*P12;
                
%                 disp('e1 is '); disp(e1);
%                 disp('e2 is '); disp(e2);
%                 disp('e3 is '); disp(e3);
%                 disp('e4 is '); disp(e4);
%                 disp('J1 is '); disp(J1);
%                 disp('J2 is '); disp(J2);
%                 disp('J3 is '); disp(J3);
%                 disp('J4 is '); disp(J4);
%                 disp('N1 is '); disp(N1);
%                 disp('N12 is '); disp(N12);
                
                % Computing the needed velocity for update
                dq1 = pJ1*e1;
                dq2 = dq1 + pinv(J2*P1,pinvtol)*(e2 - J2*dq1);
                dq3 = dq2 + pinv(J3*P12,pinvtol)*(e3 - J3*dq2);
                dq4 = dq3 + pinv(J4*P123,pinvtol)*(e4 - J4*dq3);
                
                % Updating the position
                dqnow = dq4;
                q_new = obj.sig_act + dqnow*integration_step;
                obj.set_config(q_new);
                
                error_term = [e1; e2]; % The tasks 3 and 4 are not important                
                ne = norm(error_term);
%                 disp('The norm of error is '); disp(ne);

                ntry = ntry+1;                
            end
        end
        
        % Function for get the pre-grasp configuration
        % TODO: REWRITE THIS!!!
        function q = get_starting_config(obj, cp, n, co)
            % TODO: An explanatory image for a better understanding!
            
            % Average between the two normals
            t = 0.5;
            nc = n(2,:)*t + n(1,:)*(1-t);
            
            % If the normals are the opposite, choose an orthogonal
            % projection of it (z axis direction)
            if (norm(nc) < 0.001)
%                 x_rand = rand(3,1);
                x_rand = (cp(1,:) - co).';
                if (isequal(n(1,:),-n(2,:)))
                    x_tmp = (eye(3) - n(2,:).'*n(2,:)) * x_rand;
                end
                nc = -x_tmp.';
            end
            nc = nc / norm(nc);
            
            % Find the second axis direction (y as difference between
            % contact points)
            yc = (cp(1,:) - cp(2,:)) / norm(cp(1,:) - cp(2,:));
            
            % Orthogonalize yc wrt nc (Gram–Schmidt)
            ytmp = yc - (dot(nc,yc)/dot(nc,nc))*nc;
            yc = ytmp/norm(ytmp);
            
            % Find the third axis direction as cross product
            xc = cross(yc,nc);
            xc = xc/(norm(xc));
            
            % Build rotation matrix (transpose is needed as we have the
            % columns of the new basis in the old basis)
            R = [xc' yc' nc'];
%            	rpy_ini = rotm2eul(R, 'zyx');

            % Position of the hand
            pc = cp(2,:) + 0.5*(cp(1,:) - cp(2,:)) -3*nc;
            
            % Homogenous transform for the wrist
            Xd = trvec2tform(pc)*rotm2tform(R);
            
            % Setting the config vector. A minus in the rpy is needed!.
            % Don't know why. But it works... Ask manuel to know why!
            [q, ne] = obj.compute_simple_inverse_kinematics(Xd,obj.ee_names{3});
            
            disp(ne);
            
            % For debugging
%             disp(det(R));
%             disp(R);
%             quiver3(pc(3), pc(1), pc(2), R(3,1), R(1,1), R(2,1), 'linewidth', 3.0, 'Color', [1 0 0]);
%             quiver3(pc(3), pc(1), pc(2), R(3,2), R(1,2), R(2,2), 'linewidth', 3.0, 'Color', [0 1 0]);
%             quiver3(pc(3), pc(1), pc(2), R(3,3), R(1,3), R(2,3), 'linewidth', 3.0, 'Color', [0 0 1]);
%             disp(-rpy_ini);
            
        end
        
        % Function for getring the releasing configuration
        function sig = get_release_config(obj, cp, n, co)
            % As of now using get_starting_config_george to do this...
            % TODO: a better implementation
            sig = obj.get_starting_config(cp,n,co);
            
        end
        
        % Functions for checking collisions
        % TODO: Implement this for the Franka Emika Panda!!!
        function bool = check_collisions(obj, env, points)
        % Check collisions of all joints with the set of objects.
            if ~exist('points', 'var')
                points = 10; % number of sample points in a link
            end
            % For every object in the environment:
            for i = 1:size(env, 2)
                for j = 6:8
                    bool = obj.check_collisions_joint(env{i}, j);
                    if bool == true
                        return
                    end
                end
                % check collisions of a point in a link
                % sampling some points
                for j = 6:8
                    bool = obj.check_collisions_link(env{i}, j, j+1, ...
                        points);
                    if bool == true
                        return
                    end
                end
                bool = obj.check_collisions_link(env{i}, 6, 10, points);
                if bool == true
                    return
                end
            end
            bool = false;
        end
        function bool = check_collisions_joint(obj, box, j)
            if j > size(obj.T_all, 3)
                error(['The joint number ', string(j), ...
                ' doesn''t exist'])
            end
            p = obj.T_all(1:3, 4, j);
            bool = check_collisions_point(box, p);
        end
        function bool = check_collisions_link(obj, box, j, k, points)
            if j > size(obj.T_all, 3)
                error(['The joint number ', string(j), ...
                ' doesn''t exist'])
            end
            if k > size(obj.T_all, 3)
                error(['The joint number ', string(k), ...
                ' doesn''t exist'])
            end
            p1 = obj.T_all(1:3, 4, j);
            p2 = obj.T_all(1:3, 4, k);
            bool = check_collisions_line(box, p1, p2, points);
        end
    end
end