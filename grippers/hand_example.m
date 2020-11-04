classdef hand_example < handle
    properties (Access = public)
        n_dof;
        l;
        T_all;
        q;
        X;
        J;
        Ja;
        pinvj;
        n_contacts;
        k_joints;
        k_contacts;
        S;
    end
    methods
        function obj = hand_example(link_dimensions, k_joint, k_contact)
            obj.n_dof = 8;
            if (~exist('link_dimensions', 'var') || isequal([4,1], ...
                    size(link_dimensions)))
                link_dimensions = ones(4,1);
            end
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
            obj.set_config(obj.q);
            obj.k_contacts = zeros(obj.n_contacts*6,obj.n_contacts*6);
            for i=1:obj.n_contacts
                r_indexes = [(i-1)*3+1 : i*3 obj.n_contacts*3+ ...
                    ((i-1)*3+1 : i*3)];
                obj.k_contacts(r_indexes,r_indexes) = k_contact;
            end
            obj.S = eye(obj.n_dof);
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
%             obj.compute_jacobian_analytic();
            obj.compute_pos_jacobian_inv();
        end
        function plot(obj)
            hold on;
            plot_csys(obj.T_all(:,:,6));
            hold on;
            r = 0.1;
            plot_link(obj.T_all(:,:,6), obj.T_all(:,:,7), r, false, ...
                [0 0 0]);
            hold on;
            plot_link(obj.T_all(:,:,7), obj.T_all(:,:,8), r, false, ...
                [0 13/255 73/255],'x');
            hold on;
            plot_link(obj.T_all(:,:,8), obj.T_all(:,:,9), r, true, ...
                [0 13/255 73/255], 'x');
            hold on;
            plot_link(obj.T_all(:,:,6), obj.T_all(:,:,10), r, true, ...
                [0 0 0]);
            plot_csys(obj.T_all(:,:,9), .5);
            plot_csys(obj.T_all(:,:,10), .5);
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
        function J = get_jacobian(obj)
            J = obj.J;
        end
        function compute_jacobian_analytic(obj)
        % disabled function
            % it is implemented for 'zxy' euler angles
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
        function ne = compute_differential_inverse_kinematics_george(obj, X, ...
                enable_contact, integration_step, try_max , tol, ...
                lambda_damping)
            if(~isequal([4 4 3], size(X)))
                fprintf(['Not correct configuration vector. ', ...
                    'It must be dimension [4 4 3]\n']);
                return;
            end
            if ~exist('try_max', 'var')
                try_max = 10000; % Max of iterations allowed in diff inv kin
            end
            if ~exist('enable_contact', 'var')
                enable_contact = [1;1];
                % Max of iterations allowed in diff inv kin
            end
            if ~exist('lambda_damping', 'var')
                lambda_damping = 1; % damping factor for Jacobian inverse
            end
            if ~exist('integration_step', 'var')
                integration_step = 1/100; % integration step for diff inv kin
            end
            if ~exist('tol', 'var')
                tol = .01; % Tolerance to define if a target is reached
            end
            ntry = 1;  ne = inf;
            while ntry < try_max && ne > tol
                e1 = -(obj.X(1:3,4,1) - X(1:3,4,1))*enable_contact(1);
                e2 = -(obj.X(1:3,4,2) - X(1:3,4,2))*enable_contact(2);
                error = [e1(1:3); e2(1:3)];
                % Jpinvi = obj.jacobian_inv(J_local, lambda_damping);
                % TODO. To add exploration of null space
                % P = eye(8) - Jpinvi*J_local;
%                 qp = pinv(obj.J(1:6,:))*error;
                Jnow = obj.J(1:6,:);
                qp = pinv(Jnow.'*Jnow + lambda_damping*diag(diag(Jnow.'*Jnow)))*Jnow.'*error;
                disp('Error is '); disp(error);
                disp('Mat LM is '); disp(pinv(Jnow.'*Jnow + lambda_damping*diag(diag(Jnow.'*Jnow)))*Jnow.');
                disp('qp is '); disp(qp);
                q_new = obj.q + qp*integration_step;
                obj.set_config(q_new);
                ntry = ntry+1;
                ne = norm(error);
                disp('The norm of error is '); disp(ne);
%                 disp('The present qp is '); disp(qp);
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
        function q = get_starting_config_george(obj, cp, n)
            % Average between the two normals
            t = 0.5;
            nc = n(2,:)*t + n(1,:)*(1-t);
            
            % If the normals are the same or opposite, choose an orthogonal
            % projection of it (z axis direction)
            if (norm(nc) < 0.001 || isequal(n(1,:),n(2,:)))
                x_rand = rand(3,1);
                x_tmp = (n(2,:).'*n(2,:)) * x_rand;
                nc = -x_tmp.';
            end
            nc = nc / norm(nc);
            
            % Find the second axis direction (y as difference between
            % contact points)
            yc = (cp(2,:) - cp(1,:)) / norm(cp(2,:) - cp(1,:));
            if ( abs(acos(dot(nc,yc))*180/pi) ~= 90 && ...
                    abs(acos(dot(nc,yc))*180/pi) ~= 270)
                ytmp = nc-yc/(yc*nc.');
                yc = ytmp/norm(ytmp);
            end
            
            % Find the third axis direction as cross product
            xc = cross(yc,nc);
            xc = xc / (norm(xc));
            
            % Build rotation matrix (transpose is needed as we have the
            % columns of the new basis in the old basis)
            R = [xc' yc' nc'].';
           	rpy_ini = rotm2eul(R, 'zyx');
%             disp(det(R));
%             disp(R);

            % Position of the hand
            pc = cp(2,:) + 0.5*(cp(1,:) - cp(2,:)) -2.5*nc;
%             quiver3(pc(3), pc(1), pc(2), R(3,1), R(1,1), R(2,1), 'linewidth', 3.0, 'Color', [1 0 0]);
%             quiver3(pc(3), pc(1), pc(2), R(3,2), R(1,2), R(2,2), 'linewidth', 3.0, 'Color', [0 1 0]);
%             quiver3(pc(3), pc(1), pc(2), R(3,3), R(1,3), R(2,3), 'linewidth', 3.0, 'Color', [0 0 1]);
%             disp(-rpy_ini);
            
            q = [pc -rpy_ini 0 0]'; % a minus in the rpy is needed
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
    end
end