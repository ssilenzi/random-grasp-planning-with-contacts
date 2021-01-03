classdef franka_emika_panda < matlab.mixin.Copyable
    properties (Access = public)
        rob_model;          % robot model from Robotic System Toolbox
        n_dof;              % no. of degrees of freedom
        n_links;            % no. of links
        n_ees;              % no. of end-effectors
        n_contacts;         % no. of max finger contacts
        frame_names;        % array with names of all link frames
        ee_names;           % array with names of ee (left, right, wrist)
        home_config;        % home joint configuration of the robot        
        lo_joint_lims;      % lower joint limits
        up_joint_lims;      % upper joint limits
        safe_marg           % safety margin from limits for unil. constr.
        q;                  % present joint configuration
        X;                  % FK in hom. mats. (left, right, wrist)
        T_all;              % FK in hom. mats. to all frames
        J;                  % Jacobian to the fingers left and right
        J_w;                % Jacobian to the wrist
        sig;                % sigma underact variable
        S;                  % synergy matrix
        coll_arr;           % Array containing rob. coll. geometries
        self_coll_arr;      % As above, without extremities for self coll.
        
    end
    methods
        % Constructor
        function obj = franka_emika_panda()
            % Loading the robot using Robotic System Toolbox
            obj.rob_model = importrobot('franka_gripper.urdf');
            obj.rob_model.DataFormat = 'column';
            
            % Setting constant parameters
            obj.n_dof = numel(homeConfiguration(obj.rob_model));
            % TODO: get joint limits from rigid body tree
            obj.lo_joint_lims = ...
                [-2.897, -1.762, -2.897, -3.071, -2.897, -0.017, -2.897, ...
                0.0, 0.0].'; % from franka manual
            obj.up_joint_lims = ...
                [2.897, 1.762, 2.897, -0.069, 2.897, 3.752, 2.897, ...
                0.04, 0.04].'; % from franka manual
            obj.safe_marg = 0.005;
            obj.n_links = length(obj.rob_model.Bodies); % not considering panda_link0 (base)
            obj.n_ees = 3; % left, right, wrist
            obj.n_contacts = 2;
            obj.frame_names = {};
            for k = 1:obj.n_links
                body_k = obj.rob_model.Bodies{k};
                obj.frame_names{k} = body_k.Name;
            end
            obj.ee_names = {'panda_leftfinger_tip', 'panda_rightfinger_tip', 'panda_hand'};
            obj.home_config = homeConfiguration(obj.rob_model);
            obj.home_config(8:9) = obj.up_joint_lims(8:9) - 2*obj.safe_marg; % opening hand
            
            % Initializing the variables
            obj.q = obj.home_config;
            obj.X = zeros(4,4,3);
            obj.T_all = zeros(4,4,obj.n_links);
            obj.J = [zeros(6, obj.n_dof);
                zeros(6, obj.n_dof)];
            obj.J_w = zeros(6, obj.n_dof);
            
            % The underactuation variables
            obj.sig = obj.q(1:8);
            obj.S = eye(length(obj.q),length(obj.sig)); 
            obj.S(length(obj.q),length(obj.sig)) = 1;

            % Setting the configuration and updating every variable
            set_sig(obj, obj.sig);
            
            % Getting the collision arrays from the robot model
            obj.coll_arr = ...
                exampleHelperManipCollisionsFromVisuals(obj.rob_model);
            % For self collision
            obj.self_coll_arr = obj.coll_arr;
            for j = 11:15
                obj.self_coll_arr(j,:) = obj.self_coll_arr(10,:);
            end            
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
            Jp_w = obj.J_w(1:3,:);
        end
                      
        % Set configuration function
        function set_config(obj, q_in)
            % q is a vector of 11 doubles (checking dimensions)
            if(length(q_in) ~= obj.n_dof)
                fprintf(['Not correct configuration vector. ', ...
                    'It must be dimension %d\n'], obj.n_dof);
                return;
            end
            obj.q = q_in;
            obj.compute_forward_kinematics();
            obj.compute_jacobian();
            obj.compute_wrist_jacobian();
        end
        
        % Set underact function
        function set_sig(obj, sig_in)
            obj.sig = sig_in;
            q_n = obj.S * sig_in;
            set_config(obj, q_n);
        end
        
        % Function for plotting the robot using show
        function hr = plot(obj, show_frames, pres_plot, ax)
            
            if ~exist('show_frames', 'var')
                show_frames = 'on'; % Draw also robot frames
            end
            if ~exist('pres_plot', 'var')
                pres_plot = true; % Do not preserve the prev. robot
            end
            if ~exist('ax', 'var')
                hr = show(obj.rob_model, obj.q, ...
                    'visuals', 'on', ...
                    'Frames', show_frames, ...
                    'PreservePlot', pres_plot);
            else
                hr = show(obj.rob_model, obj.q, ...
                    'visuals', 'on', ...
                    'Frames', show_frames, ...
                    'PreservePlot', pres_plot, 'Parent', ax);
            end

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
        function [q_sol, success] = compute_simple_inverse_kinematics(obj, Xd, ...
                frame_name, weights_joints)
            
            % This inverse kinematics is based on the default RSToolbox
            % Given the desired pose and the frame name, computes the
            % joints needed to reach that pose with the link
            % This is not possible for finger frames now (no underact)
            
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
                weights_joints = 5*ones(size(obj.q)); % Weights on q
            end
            
            % Saving the starting finger joints for later use and checking
            % if target frame is related to finger
            q_fing = obj.q(8:9);
            fing_fr = ind > 10;
            
            if fing_fr
                disp('The requested frame is '); disp(frame_name);
                error(['Cannot do ik for finger frame here!', ...
                    ' Use compute_differential_inverse_kinematics']);
            end
            
            % Setting generalized ik solver
            gik = generalizedInverseKinematics('RigidBodyTree', ...
                obj.rob_model, ...
                'ConstraintInputs', {'pose','joint'});
            
            % Setting the des pose constraint
            desPose = constraintPoseTarget(frame_name);
            desPose.ReferenceBody = obj.frame_names{1}; % base frame
            desPose.TargetTransform = Xd; % desired pose
            desPose.PositionTolerance = 0.01;
            desPose.OrientationTolerance = 0.01;
            
            % Setting the joint limits constraint
            limitJoint = constraintJointBounds(obj.rob_model);
            limitJoint.Bounds = [obj.lo_joint_lims, obj.up_joint_lims];
            limitJoint.Weights = weights_joints.';
            
            % Solving ik
            [q_sol, ~] = gik(obj.q, desPose, limitJoint);
            
            % Resetting finger joints if needed and setting config
            if ~fing_fr
                q_sol(8:9) = q_fing;
            end
            obj.set_config(q_sol);
            
            % Computing error
            error_now = Xd - obj.T_all(:,:,ind);
            ne = norm(error_now(1:3,4))
            
            % Checking also joint limits
            is_viol_joints = ...
                any(obj.q < obj.lo_joint_lims) || ...
                any(obj.q > obj.up_joint_lims);
            
            success = ne < 0.1 && ~is_viol_joints;
            
        end

        % Inverse Kinematics function using Stack of Tasks
        function ne = compute_differential_inverse_kinematics(obj, ...
                Xd, is_unil, unil_ind, is_min, sig_now, enable_contact, ...
                integration_step, try_max , tol, pinvtol)
            
            % This differential IK is priority based inversion for the
            % three desired poses of finger-tips and wrist.
            % As of now, higher priorities given to fingers and least to
            % the wrist.
            % [Siciliano Slotine, A General Framework for managing ...]
            % This function also works for unilateral constraints on the
            % gripper joint:
            % [Mansard, A unified approach to integrate unilateral...]
            % HOW TO USE:
            % If a wrist pose is present in Xd(:,:,3) -> TASK 3
            % If is_unil -> Unil. constr. for the robot joints - TASK 4,5
            % If is_min -> Nearest sol. to provided q_now - TASK 6
            
            % Here underactuation is used as the fingers of the franka
            % gripper seem not to be able to move independently
            
            if ~exist('is_unil', 'var')
                is_unil = false; % For unilateral constr. task
            end
            if is_unil && ~exist('unil_ind', 'var')
                error('Please specify the indexes of the unilateral vars!');
            elseif is_unil && exist('unil_ind', 'var')
               if any(unil_ind < 0) || any(~(floor(unil_ind) == unil_ind))
                   error('The unil_ind must have only natural numbers!');
               end
               if max(unil_ind) > length(obj.sig)
                   error('The max value of unil_ind is greater than size sig!');
               end
               if any(diff(unil_ind) == 0)
                   error('I need consecutive values in unil_ind!');
               end
            end
            if ~exist('is_min', 'var')
                is_min = false; % For min. joint variation task
            end
            if is_min == true && ~exist('sig_now', 'var') % Throw error
                error('The sigma value was not provided with is_min!'); 
            end
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
                tol = .005; % Tolerance to define if a target is reached
            end
            if ~exist('pinvtol', 'var')
                pinvtol = .01; % Tolerance for "cut-off" pinv
            end
            if ~exist('lam_unil', 'var')
                lam_unil = 0.1; % Velocity for unilateral constraints
            end
            
            % Checking if there is also a wrist reference
            is_wrist = false;
            if(isequal([4 4 3], size(Xd)))
                is_wrist = true;
            end
            
            ntry = 1;  ne = inf; dsig_upd = zeros(size(obj.sig));
            while ntry < try_max && ne > tol
                
                % Unil constrs for joint limits have highest priority
                if is_unil  % UC on finger joints
                    % errors
                    e_u = obj.up_joint_lims(unil_ind) - obj.safe_marg - obj.sig(unil_ind); % upper limit
                    e_l = obj.sig(unil_ind) + obj.safe_marg - obj.lo_joint_lims(unil_ind); % lower limit
                    e1 = e_u;
                    e2 = e_l;
                    lvec1 = e1 <= 0 & dsig_upd(unil_ind) >= 0;
                    lvec2 = e2 <= 0 & dsig_upd(unil_ind) <= 0;                    
                    e1 = zeros(length(unil_ind),1);
                    e2 = zeros(length(unil_ind),1);
                    
                    % H matrices and jacobians
                    H1 = zeros(length(unil_ind),length(obj.sig));
                    H1(:,unil_ind) = diag(lvec1);                   
                    J1 = lam_unil * H1;
                    H2 = zeros(length(unil_ind),length(obj.sig));
                    H2(:,unil_ind) = diag(lvec2);                  
                    J2 = lam_unil * H2;
                else
                    % zero errors and jacobians
                    e1 = zeros(length(obj.sig),1);
                    J1 = zeros(length(obj.sig));
                    e2 = zeros(length(obj.sig),1);
                    J2 = zeros(length(obj.sig));
                end
                
                % Compute the position error and the jacobians 
                % for tasks 1 and 2
                e3 = (Xd(1:3,4,1) - obj.X(1:3,4,1))*enable_contact(1); % left finger
                e4 = (Xd(1:3,4,2) - obj.X(1:3,4,2))*enable_contact(2); % right finger
                [J3, J4, ~] = obj.get_pos_jacobians();
                J3 = J3*obj.S;
                J4 = J4*obj.S;
                
                % Now computing task 3 according to the flags
                % If there is a wrist ref. then get a task 4
                if is_wrist
                    e5 = (Xd(1:3,4,3) - obj.X(1:3,4,3));
                    [~, ~, J5] = obj.get_pos_jacobians();
                else
                    e5 = zeros(3,1);
                    J5 = zeros(3,length(obj.q));
                end
                J5 = J5*obj.S;
                                               
                if is_min % For minimum variation from q_now
                    % errors and jacobians
                    e6 = sig_now - obj.sig;
                    J6 = eye(length(obj.sig));
                else
                    e6 = zeros(length(obj.sig),1);
                    J6 = zeros(length(obj.sig));
                end
                                                              
                % Computing pseudo-invs and projectors
                pJ1 = pinv(J1,pinvtol);
                P1 = (eye(size(obj.sig,1)) - pJ1*J1);
                P12 = P1 - pinv(J2*P1,pinvtol)*J2*P1;
                P123 = P12 - pinv(J3*P12,pinvtol)*J3*P12;
                P1234 = P123 - pinv(J4*P123,pinvtol)*J4*P123;
                P12345 = P1234 - pinv(J5*P1234,pinvtol)*J5*P1234;
                
%                 disp('e1 is '); disp(e1);
%                 disp('e2 is '); disp(e2);
%                 disp('e3 is '); disp(e3);
%                 disp('e4 is '); disp(e4);
%                 disp('J1 is '); disp(J1);
%                 disp('J2 is '); disp(J2);
%                 disp('J3 is '); disp(J3);
%                 disp('J4 is '); disp(J4);
                
                % Computing the needed velocity for update
                dsig1 = pJ1*e1;
                dsig2 = dsig1 + pinv(J2*P1,pinvtol)*(e2 - J2*dsig1);
                dsig3 = dsig2 + pinv(J3*P12,pinvtol)*(e3 - J3*dsig2);
                dsig4 = dsig3 + pinv(J4*P123,pinvtol)*(e4 - J4*dsig3);
                dsig5 = dsig4 + pinv(J5*P1234,pinvtol)*(e5 - J5*dsig4);
                dsig6 = dsig5 + pinv(J6*P12345,pinvtol)*(e6 - J6*dsig5);
                
                % Updating the position
                dsig_upd = dsig6;
                sig_new = obj.sig + dsig_upd*integration_step;
                obj.set_sig(sig_new);
                
                error_pos = [e3; e4]; % The tasks 3 and 4 are considered                
                ne = norm(error_pos);
%                 disp('The norm of error is '); disp(ne);

                ntry = ntry+1;                
            end
        end
        
        % Function for get the pre-grasp configuration
        % TODO: REWRITE THIS!!!
        function [q_start, ne] = get_starting_config(obj, cp, n, co, box)
            % TODO: An explanatory image for a better understanding!
            
            % Getting a scale factor using the dimensions of the box if it
            % is provided
            if ~exist('box','var')
                scale = 0.1;
            else
                scale = sqrt(box.l^2 + box.h^2 + box.w^2);
            end
            
            % Average between the two normals
            t = 0.5;
            nc = n(2,:)*t + n(1,:)*(1-t);
            
            % If the normals are the opposite, choose an orthogonal
            % projection of it (z axis direction)
            if (norm(nc) < 0.001)
                %                 x_rand = rand(3,1);
                x_rand = (cp(1,:) - co).';
%                 x_tmp = (eye(3) - n(2,:).'*n(2,:)) * x_rand;
                % Gram–Schmid x_rand wrt n2
                x_tmp = x_rand - (dot(n(2,:).',x_rand)/dot(n(2,:).',n(2,:).'))*n(2,:).';
                nc = -x_tmp.';
            end
            nc = scale * nc / norm(nc);
            
            % Find the second axis direction (y as difference between
            % contact points)
            yc = (cp(1,:) - cp(2,:)) / norm(cp(1,:) - cp(2,:));
            
            % Orthogonalize yc wrt nc (Gram–Schmidt)
            ytmp = yc - (dot(nc,yc)/dot(nc,nc))*nc;
            yc = scale * ytmp/norm(ytmp);
            
            % Find the third axis direction as cross product
            xc = cross(yc,nc);
            xc = scale * xc/(norm(xc));
            
            % Build rotation matrix (transpose is needed as we have the
            % columns of the new basis in the old basis)
            xcn = xc / norm(xc);
            ycn = yc / norm(yc);
            ncn = nc / norm(nc);
            R = [xcn' ycn' ncn'];
%            	rpy_ini = rotm2eul(R, 'zyx');

            % Position of the hand
            pc = cp(2,:) + 0.5*(cp(1,:) - cp(2,:)) -1.5*nc;
            
            % Homogenous transform for the wrist
            Xd = trvec2tform(pc)*rotm2tform(R);
            
%             disp('det R '); disp(det(R));
%             disp('Initial q is '); obj.q;
            
            % Performing IK and setting the config vector.             
            [q_start, ne] = obj.compute_simple_inverse_kinematics(Xd,obj.ee_names{3});
            q_start(8:9) = obj.home_config(8:9); % opening hand
            
%             disp('Found q is '); q_start;
%             disp('Final error is '); disp(ne);
                                    
            % For debugging
%             disp(det(R));
%             disp(R);
            quiver3(pc(1), pc(2), pc(3), xc(1), xc(2), xc(3), 'linewidth', 3.0, 'Color', [1 0 0]);
            quiver3(pc(1), pc(2), pc(3), yc(1), yc(2), yc(3), 'linewidth', 3.0, 'Color', [0 1 0]);
            quiver3(pc(1), pc(2), pc(3), nc(1), nc(2), nc(3), 'linewidth', 3.0, 'Color', [0 0 1]);
            
        end
        
        % Function for getring the releasing configuration
        function sig = get_release_config(obj, cp, n, co)
            % As of now using get_starting_config_george to do this...
            % TODO: a better implementation
            sig = obj.get_starting_config(cp,n,co);
            
        end
        
        % Functions for checking collisions
        function bool = check_box_collisions(obj, env)
            % Check collisions of robot with the set of boxes.
            
        end
        function [bool_col, self_coll_pair_id] = check_self_collisions(obj)
            % Check self collisions of robot.
            [bool_col, self_coll_pair_id] = ...
                exampleHelperManipCheckCollisions(obj.rob_model, ...
                obj.self_coll_arr, {}, obj.q, true);
        end
        
    end
end