classdef hand_example < matlab.mixin.Copyable
    properties (Access = public)
        n_dof;
        l;
        T_all;
        q;
        sig_act;
        X;
        J;
        J_wrist;
        Ja;
        pinvj;
        n_contacts;
        k_joints;
        k_contacts;
        S;
    end
    methods
        function obj = hand_example(link_dimensions, is_und, k_joint, k_contact)
            if (~exist('link_dimensions', 'var') || ~isequal([4,1], ...
                    size(link_dimensions)))
                link_dimensions = ones(4,1);
            end
            if (~exist('is_und', 'var'))
                is_und = false;
            end
            obj.n_dof = 8; % degrees of freed. is always 8 (doa can change)
            if ~exist('k_joint', 'var')
                obj.k_joints = diag(100*ones(obj.n_dof,1));
            else
                obj.k_joints = diag(k_joint*ones(obj.n_dof,1));
            end
            if ~exist('k_contact', 'var')
                k_contact = diag([1002,1003,1004,105,106,107]);
            else
                k_contact = diag(k_contact);
            end
            obj.l = link_dimensions;
            obj.n_contacts = 2;
            obj.T_all = zeros(4,4,10);
            obj.q = zeros(obj.n_dof,1);
            obj.k_contacts = zeros(obj.n_contacts*6,obj.n_contacts*6);
            for i=1:obj.n_contacts
                r_indexes = [(i-1)*3+1 : i*3 obj.n_contacts*3+ ...
                    ((i-1)*3+1 : i*3)];
                obj.k_contacts(r_indexes,r_indexes) = k_contact;
            end
            % Setting the synergy matrix and the actuation var
            if is_und
                obj.S = eye(obj.n_dof); 
                obj.S(:,end) = []; obj.S(end,end) = 1;
                obj.sig_act = obj.q(1:7);
            else
                obj.S = eye(obj.n_dof);
                obj.sig_act = obj.q;
            end
            set_act(obj, obj.sig_act);
        end
        function n = get_n_dof(obj)
            n = obj.n_dof;
        end
        function n = get_n_contacts(obj)
            n = obj.n_contacts;
        end
        function T = get_T_all(obj)
            T = obj.T_all;
        end
        function set_act(obj, sig_act)
            % setting the actuation vector and then the hand config
            obj.sig_act = sig_act;
            set_config(obj, obj.S*obj.sig_act);
        end
        function set_config(obj, q)
            % q is defined as a vector of x y z ax ay az
            if(length(q) ~= obj.n_dof)
                fprintf(['Not correct configuration vector. ', ...
                    'It must be dimension %d\n'], obj.n_dof);
                return;
            end
            obj.q = q;
            obj.compute_forward_kinematics();
            obj.compute_jacobian();
            obj.compute_wrist_jacobian();
%             obj.compute_jacobian_analytic();
            obj.compute_pos_jacobian_inv();
        end
        function hr = plot(obj)
            hold on;
            hr(1:3) = plot_csys(obj.T_all(:,:,6));
            hold on;
            r = 0.1;
            hr(4:7) = plot_link(obj.T_all(:,:,6), obj.T_all(:,:,7), r, false, ...
                [0 0 0]);
            hold on;
            hr(8:11) = plot_link(obj.T_all(:,:,7), obj.T_all(:,:,8), r, false, ...
                [0 13/255 73/255],'x');
            hold on;
            hr(12:15) = plot_link(obj.T_all(:,:,8), obj.T_all(:,:,9), r, true, ...
                [0 13/255 73/255], 'x');
            hold on;
            hr(16:19) = plot_link(obj.T_all(:,:,6), obj.T_all(:,:,10), r, true, ...
                [0 0 0]);
            hr(20:22) = plot_csys(obj.T_all(:,:,9), .5);
            hr(23:25) = plot_csys(obj.T_all(:,:,10), .5);
            xlabel('z');
            ylabel('x');
            zlabel('y');
        end
        function compute_forward_kinematics(obj)
            T_all_local= zeros(4,4,10);
            % csys to hand base
            T_all_local(:,:,1) = transl(obj.q(1),0,0);
            T_all_local(:,:,2) = T_all_local(:,:,1) * transl(0,obj.q(2),0);
            T_all_local(:,:,3) = T_all_local(:,:,2) * transl(0,0,obj.q(3));
            T_all_local(:,:,4) = T_all_local(:,:,3) * trotz(obj.q(4));
            T_all_local(:,:,5) = T_all_local(:,:,4) * troty(obj.q(5));
            T_all_local(:,:,6) = T_all_local(:,:,5) * trotx(obj.q(6)); 
            % csys to end effector
            T_all_local(:,:,7) = T_all_local(:,:,6) * ...
                transl(0, obj.l(1), 0); % p1
            T_all_local(:,:,8) = T_all_local(:,:,7) * trotx(obj.q(7)) * ...
                transl(0, obj.l(2), 0); % p2
            T_all_local(:,:,9) = T_all_local(:,:,8) * trotx(obj.q(8)) * ...
                transl(0, obj.l(3), 0); % p3, Cp1
            T_all_local(:,:,10) = T_all_local(:,:,6) * ...
                transl(0, 0, obj.l(4)); % p4, Cp2
            % This Rotx of pi/2 is needed to have finger frame
            % y normal to the contact
            T_all_local(:,:,10) = T_all_local(:,:,10) * trotx(-pi/2);
            % output fk of 9, 10, 6
            obj.T_all = T_all_local;
            obj.X(:,:,1) = T_all_local(:,:,9);
            obj.X(:,:,2) = T_all_local(:,:,10);
            obj.X(:,:,3) = T_all_local(:,:,6);
        end
        function X = get_forward_kinematics(obj)
            X = obj.X;
        end
        function compute_jacobian(obj)
            % This function computes the geometrical Jacobian
            Cp = [ obj.T_all(1:3,4,9).';
                  obj.T_all(1:3,4,10).'];
            Org1 = [0, 0, 0];
            Org2 = obj.T_all(1:3,4,1).';
            Org3 = obj.T_all(1:3,4,2).';
            Org4 = obj.T_all(1:3,4,3).';
            Org5 = obj.T_all(1:3,4,4).';
            Org6 = obj.T_all(1:3,4,5).';
            Org7 = obj.T_all(1:3,4,7).';
            Org8 = obj.T_all(1:3,4,8).';
            Org = [Org1;
                   Org2;
                   Org3;
                   Org4;
                   Org5;
                   Org6;
                   Org7;
                   Org8];
            Zax = [1, 0, 0;
                   0, 1, 0;
                   0, 0, 1;
                   0, 0, 1;
                   (obj.T_all(1:3,1:3,4)*[0; 1; 0]).';
                   (obj.T_all(1:3,1:3,5)*[1; 0; 0]).';
                   (obj.T_all(1:3,1:3,5)*[1; 0; 0]).';
                   (obj.T_all(1:3,1:3,5)*[1; 0; 0]).'];
            
            obj.J = build_jt(Cp, Org, Zax, ...
                             [2, 2, 2, 1, 1, 1, 1, 1;
                              2, 2, 2, 1, 1, 1, 0, 0]...
                            ).'; % J is the transposed of Jt
        end
        function compute_wrist_jacobian(obj)
            % This function computes the geometric Jacobian of the wrist
            Cp = obj.T_all(1:3,4,6).';
            Org1 = [0, 0, 0];
            Org2 = obj.T_all(1:3,4,1).';
            Org3 = obj.T_all(1:3,4,2).';
            Org4 = obj.T_all(1:3,4,3).';
            Org5 = obj.T_all(1:3,4,4).';
            Org6 = obj.T_all(1:3,4,5).';
            Org7 = zeros(1,3);
            Org8 = zeros(1,3);
            Org = [Org1;
                   Org2;
                   Org3;
                   Org4;
                   Org5;
                   Org6
                   Org7
                   Org8];
            Zax = [1, 0, 0;
                   0, 1, 0;
                   0, 0, 1;
                   0, 0, 1;
                   (obj.T_all(1:3,1:3,4)*[0; 1; 0]).';
                   (obj.T_all(1:3,1:3,5)*[1; 0; 0]).';
                   (obj.T_all(1:3,1:3,5)*[1; 0; 0]).';
                   (obj.T_all(1:3,1:3,5)*[1; 0; 0]).'];
            
            obj.J_wrist = build_jt(Cp, Org, Zax, ...
                             [2, 2, 2, 1, 1, 1, 0, 0]).'; % J is the transposed of Jt
        end
        function J = get_jacobian(obj)
            J = obj.J;
        end
        function Jw = get_wrist_jacobian(obj)
            Jw = obj.J_wrist;
        end
        function [Jp1, Jp2, Jpw] = get_pos_jacobians(obj)
            J_tmp = obj.J;
            Jp1 = J_tmp(1:3,:);
            Jp2 = J_tmp(4:6,:);
            Jpw = obj.J_wrist(1:3,:);
        end
        function [Jp1, Jp2, Jpw] = get_pos_jacobians_from_symb(obj)
            x = obj.q(1); y = obj.q(2); z = obj.q(3);
            yaw = obj.q(4); pit = obj.q(5); rol = obj.q(6);
            q1 = obj.q(7); q2 = obj.q(8);
            l1 = obj.l(1); l2 = obj.l(2); l3 = obj.l(3); l4 = obj.l(4);
            [Jp1, Jp2, Jpw] = hand_symbolic_jacobians(x, y, z, ...
                yaw, pit, rol, q1, q2, l1, l2, l3, l4);
        end
        function compute_jacobian_analytic(obj)
        % disabled function
            % it is implemented for 'zyx' euler angles
            Rc1 = rotz(obj.q(4))*[0;0;1];
            Rc2 = rotz(obj.q(4))*roty(obj.q(5))*[0;1;0];
            Rc3 = rotz(obj.q(4))*roty(obj.q(5))*rotx(obj.q(6))*[1;0;0];
            Ra = [Rc1 Rc2 Rc3];
            A = [eye(3), zeros(3,9);
                zeros(3), eye(3), zeros(3,6);
                zeros(3,6), inv(Ra), zeros(3);
                zeros(3,9), inv(Ra)];
            % output Ja
            obj.Ja = A * obj.J;
        end
        function Ja = get_jacobian_analytic(obj)
            Ja = obj.Ja;
        end
        function ne = compute_differential_inverse_kinematics_george(obj, Xd, sig_open_d, ...
                enable_contact, integration_step, try_max , tol, ...
                lambda_damping)
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
            % wrist) -> unilateral constraints
            % If q_open_d has eight elements (also wrist joints) -> Nearest
            % solution to q_open_d
            if ~exist('try_max', 'var')
                try_max = 100; % Max of iterations allowed in diff inv kin
            end
            if ~exist('enable_contact', 'var')
                enable_contact = [1;1];
                % Max of iterations allowed in diff inv kin
            end
            if ~exist('lambda_damping', 'var')
                lambda_damping = 1; % damping factor for Jacobian inverse
            end
            if ~exist('integration_step', 'var')
                integration_step = 1/10; % integration step for diff inv kin
            end
            if ~exist('tol', 'var')
                tol = .01; % Tolerance to define if a target is reached
            end
            
            % Checking if there is also a wrist reference
            is_wrist = false;
            if(isequal([4 4 3], size(Xd)))
                is_wrist = true;
            end
            
            % Tolerance for "cut-off" pinv
            pinvtol = 0.01;
            
            % Velocity for unilateral constraint
            lam_unil = 1;
            
            % Unilateral or minimum difference? (Only if no wrist)
            if size(sig_open_d,1) <= 2
                is_unil = true;
            elseif size(sig_open_d,1) == size(obj.sig_act,1)
                is_unil = false;
            else
                error('Do not know what to do! In IK, unexpected joints size!');
            end
            
            ntry = 1;  ne = inf;
            while ntry < try_max && ne > tol
                
                % Compute the position error and the jacobians for tasks 1
                % and 2
                e1 = (Xd(1:3,4,1) - obj.X(1:3,4,1))*enable_contact(1); % finger 1
                e2 = (Xd(1:3,4,2) - obj.X(1:3,4,2))*enable_contact(2); % finger 2
                [J1, J2, ~] = obj.get_pos_jacobians();
                % Multipy Sigma matrix
                J1 = J1*obj.S;
                J2 = J2*obj.S;
                
                % Now computing task 3 according to the flags
                % If there is a wrist ref. then get a task 4
                if is_wrist
                    e3 = (Xd(1:3,4,3) - obj.X(1:3,4,3));
                    [~, ~, J3] = obj.get_pos_jacobians();
                else
                    e3 = zeros(3,1);
                    J3 = zeros(3,size(obj.q,1));
                end
                J3 = J3*obj.S;
                
                if is_unil  % For the unilateral constraint
                    % error, H matrix and jacobian
                    if length(sig_open_d) == 1    % underact.
                        e4 = sig_open_d - obj.sig_act(7);
                        lvec = e4 <= 0;
                        H7 = lvec(1);
                        H = [zeros(1,6), H7];
                    else                            % fully act.
                        e4 = sig_open_d - obj.sig_act(7:8);
                        lvec = e4 <= 0;
                        H78 = [lvec(1), 0; 0, lvec(2)];
                        H = [zeros(2,6), H78];
                    end
                                       
                    J4 = lam_unil * H;
                else        % For minumum difference from q_open_d
                    % error and jacobian for keeping configuration
                    e4 = q_open_d - obj.q;
                    J4 = eye(size(obj.q,1));
                end
                % THERE IS NO NEED FOR MULTIPLYING J4 with Synergy Matrix
                                                              
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
                dsig1 = pJ1*e1;
                dsig2 = dsig1 + pinv(J2*P1,pinvtol)*(e2 - J2*dsig1);
                dsig3 = dsig2 + pinv(J3*P12,pinvtol)*(e3 - J3*dsig2);
                dsig4 = dsig3 + pinv(J4*P123,pinvtol)*(e4 - J4*dsig3);

                
%                 disp('Error is '); disp(error);
%                 disp('dq3 is '); disp(dq3);
                
                % Updating the position
                dsignow = dsig4;
                sig_new = obj.sig_act + dsignow*integration_step;
                obj.set_act(sig_new);
                
                error_term = [e1; e2]; % The tasks 3 and 4 are not important                
                ne = norm(error_term);
%                 disp('The norm of error is '); disp(ne);

                ntry = ntry+1;
                
            end
        end
        function compute_pos_jacobian_inv(obj, tol)
%         function pinvj = jacobian_inv(obj, Ji, lambda)
%             if ~exist('lambda', 'var')
%                 lambda = .01; % damping factor
%             end
%             if ~exist('epsilon', 'var')
%                 epsilon = 1e-3; % damping factor
%             end
%             [U,S,V] = svd(Ji);
%             for j = 1:size(S,1)
%                 if S(j,j) < epsilon
%                     S(j,j) = S(j,j) / (S(j,j)^2+lambda^2);
%                 end
%             end
%             S = S.';
%             iJJt = V*S*U.';
%             pinvj = iJJt;
            if ~exist('tol', 'var')
                tol = 1e-6;
            end
            obj.pinvj = pinv(obj.J(1:6,:), tol);
        end
        function pinvj = get_pos_jacobian_inv(obj)
            pinvj = obj.pinvj;
        end
        function q = get_starting_config(obj, cp, n)
            t = 0.5;
            nc = n(2,:)*t + n(1,:)*(1-t);
%             if (norm(nc) < 0.001)
            if (norm(nc) < 0.001 || isequal(n(1,:),n(2,:)))
                x_rand = rand(3,1);
                x_tmp = (eye(3) - n(2,:).'*n(2,:)) * x_rand;
%                 x_tmp = n(2,:)-x_rand/(x_rand*n(2,:)');
                nc = x_tmp.';
            end
            nc = nc / norm(nc);
            yc = (cp(1,:) - cp(2,:)) / norm(cp(1,:) - cp(2,:));
            if ( abs(acos(dot(nc,yc))*180/pi) ~= 90 && ...
                    abs(acos(dot(nc,yc))*180/pi) ~= 270)
                ytmp = nc-yc/(yc*nc.');
                yc = ytmp/norm(ytmp);
            end
            xc = cross(yc,nc);
            xc = xc / (norm(xc));
            R = [xc' yc' nc'];
            pc = cp(2,:) + 0.5*(cp(1,:) - cp(2,:)) + (R*[0;0;-2]).';
            rpy_ini = rotm2eul(R, 'zyx');
            q = [pc rpy_ini 1 1]';
        end
        % To get the starting configuration (tmp by George)
        function sig = get_starting_config_george(obj, cp, n, co)
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
           	rpy_ini = rotm2eul(R, 'zyx');

            % Position of the hand
            pc = cp(2,:) + 0.5*(cp(1,:) - cp(2,:)) -3*nc;
            
            % Setting the config vector. A minus in the rpy is needed!.
            % Don't know why. But it works... Ask manuel to know why!
            sig = -0.75*ones(1,size(obj.sig_act,1));
            sig(1:6) = [pc, -rpy_ini];
            sig = sig.';
            
            % To debug
%             disp(det(R));
%             disp(R);
%             quiver3(pc(3), pc(1), pc(2), R(3,1), R(1,1), R(2,1), 'linewidth', 3.0, 'Color', [1 0 0]);
%             quiver3(pc(3), pc(1), pc(2), R(3,2), R(1,2), R(2,2), 'linewidth', 3.0, 'Color', [0 1 0]);
%             quiver3(pc(3), pc(1), pc(2), R(3,3), R(1,3), R(2,3), 'linewidth', 3.0, 'Color', [0 0 1]);
%             disp(-rpy_ini);
            
        end
        % To get the releasing configuration (removal of hand contacts) (tmp by George)
        function sig = get_release_config_george(obj, cp, n, co)
            % As of now using get_starting_config_george to do this...
            % TODO: a better implementation
            sig = obj.get_starting_config_george(cp,n,co);
            
        end
        function e = diff(obj, T1, T2)
            % TODO testing;
            T_1_2 = T1\T2;
            e =  zeros(6,1);
            e(1:3,1) = T_1_2(1:3,4);
            [xi, theta] = homtotwist(T_1_2);
            e(4:6,1) = xi(4:6)*theta;
            e = ad([T1(1:3,1:3) [0;0;0]; [0 0 0 1]])*e;
        end
        function K = get_joint_contact_stiffness(obj)
            K = eye(obj.n_contacts*6);
            Cq = inv(obj.k_joints);
            for i=1:obj.n_contacts
                r_indexes = [(i-1)*3+1 : ...
                    i*3 obj.n_contacts*3+((i-1)*3+1 : i*3)];          
                A = ad([obj.X(1:3,1:3,i) [0;0;0];[0 0 0 1]]);
                Ki = obj.k_contacts(r_indexes,r_indexes);
                Ki_world = A'*Ki*A;
                Cs = inv(Ki_world);
                J_i = obj.J(r_indexes,:);
                K_world_total = inv(Cs +J_i*Cq*J_i');                          
                K(r_indexes,r_indexes) = K_world_total;
            end       
        end
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