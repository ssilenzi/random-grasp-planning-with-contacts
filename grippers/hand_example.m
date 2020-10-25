classdef hand_example < handle
    properties (Access = public)
        n_dof;
        l;
        T_all;
        q;
        X;
        J;
        Ja;
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
            obj.T_all= zeros(4,4,10);
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
                fprintf(['Not correct configuration vector. It must ', ...
                    'be dimension %d\n'], obj.n_dof);
                return;
            end
            obj.q = q;
            obj.compute_forward_kinematics();
            obj.compute_jacobian();
            obj.compute_jacobian_analytic();
        end
        function plot(obj)
            hold on;
            plot_csys(obj.T_all(:,:,6));
            hold on;
            r = 0.1;
            plot_link(obj.T_all(:,:,6),obj.T_all(:,:,7), r, false, ...
                [0 0 0]);
            hold on;
            plot_link(obj.T_all(:,:,7),obj.T_all(:,:,8), r, false, ...
                [0 13/255 73/255],'x');
            hold on;
            plot_link(obj.T_all(:,:,8),obj.T_all(:,:,9), r, true, ...
                [0 13/255 73/255], 'x');
            hold on;
            plot_link(obj.T_all(:,:,6),obj.T_all(:,:,10), r, true, ...
                [0 0 0]);
            plot_csys(obj.T_all(:,:,9),.5);
            plot_csys(obj.T_all(:,:,10),.5);
            xlabel('z');
            ylabel('x');
            zlabel('y');
        end
        function compute_jacobian(obj)
            % This function computes the geometrical Jacobian
            cp = [obj.T_all(1:3,4,9).';obj.T_all(1:3,4,10).'];
            org1 = [0 0 0];
            org2 = obj.T_all(1:3,4,1).';
            org3 = obj.T_all(1:3,4,2).';
            org4 = obj.T_all(1:3,4,3).';
            org5 = obj.T_all(1:3,4,4).';
            org6 = obj.T_all(1:3,4,5).';
            org7 = obj.T_all(1:3,4,7).';
            org8 = obj.T_all(1:3,4,8).';
            
            org = [org1;org2;org3;org4;org5;org6;org7;org8];
            zaxs = [[1 0 0];[0 1 0];[0 0 1];[0 0 1];
                (obj.T_all(1:3,1:3,4)*[0 1 0].').';
                (obj.T_all(1:3,1:3,5)*[1 0 0].').';
                (obj.T_all(1:3,1:3,6)*[1 0 0].').';
                (obj.T_all(1:3,1:3,6)*[1 0 0].').'];
            
            obj.J = build_jt(cp,org,zaxs,[2 2 2 1 1 1 1 1;
                2 2 2 1 1 1 0 0]).';
            % TODO Include the synergy
        end
        function J = get_jacobian(obj)
            J = obj.J;
        end
        function compute_forward_kinematics(obj)
            T_all_local= zeros(4,4,10);
            T_all_local(:,:,1) = transl(obj.q(1),0,0);
            T_all_local(:,:,2) = T_all_local(:,:,1)*transl(0,obj.q(2),0);
            T_all_local(:,:,3) = T_all_local(:,:,2)*transl(0,0,obj.q(3));
            T_all_local(:,:,4) = T_all_local(:,:,3)*trotz(obj.q(4));
            T_all_local(:,:,5) = T_all_local(:,:,4)*troty(obj.q(5));
            T_all_local(:,:,6) = T_all_local(:,:,5)*...
                trotx(obj.q(6)); % CSYS to hand base
            T_all_local(:,:,7) = T_all_local(:,:,6)*...
                transl(0,obj.l(1),0); %p1
            T_all_local(:,:,8) = T_all_local(:,:,7)*trotx(obj.q(7))*...
                transl(0,obj.l(2),0); %p2
            T_all_local(:,:,9) = T_all_local(:,:,8)*trotx(obj.q(8))*...
                transl(0,obj.l(3),0); %p3, Cp1
            T_all_local(:,:,10) = T_all_local(:,:,6)*...
                transl(0, 0, obj.l(4)); %p4, Cp2
            obj.T_all = T_all_local;
            obj.X(:,:,1)  = T_all_local(:,:,9);
            obj.X(:,:,2)  = T_all_local(:,:,10);
            obj.X(:,:,3)  = T_all_local(:,:,6);
        end
        function X = get_forward_kinematics(obj)
            X = obj.X;
        end
        function compute_jacobian_analytic(obj)
            % it is implemented for 'zxy' euler angles
            Rc1 = rotz(obj.q(4))*[0;0;1];
            Rc2 = rotz(obj.q(4))*roty(obj.q(5))*[0;1;0];
            Rc3 = rotz(obj.q(4))*roty(obj.q(5))*rotx(obj.q(6))*[1;0;0];
            Ra = [Rc1 Rc2 Rc3];
            A = [eye(3) zeros(3,9);
                zeros(3) eye(3) zeros(3,6);
                zeros(3,6) Ra' zeros(3);
                zeros(3,9) Ra'];
            
            obj.Ja = A*obj.J;
        end        
        function compute_inverse_kinematics(~, X)
            if(~isequal([3 3], size(X)))
                fprintf(['Not correct configuration vector. ', ...
                    'It must be dimension [2, 6]\n']);
                return;
            end
            fprintf('Not implemented yet :(\n');
        end
        function ne = compute_differential_inverse_kinematics(obj, X, ...
                enable_contact, integration_step, tryMax , TOL, ...
                lambda_damping)
            if(~isequal([4 4 3], size(X)))
                fprintf(['Not correct configuration vector. ', ...
                    'It must be dimension [4 4 3]\n']);
                return;
            end
            if ~exist('tryMax', 'var')
                tryMax = 1000; % Max of iterations allowed in difFIk
            end
            if ~exist('enable_contact', 'var')
                enable_contact = [1;1];
                % Max of iterations allowed in difFIk
            end
            if ~exist('lambda_damping', 'var')
                lambda_damping = .01; % damping factor for Jacobian inverse
            end
            if ~exist('integration_step', 'var')
                integration_step = 1/10; % dintegration spet for diffIK
            end
            if ~exist('TOL', 'var')
                TOL = .01; % Tolerance to define if a target is reached
            end
            ntry = 1;  ne = inf;
            while  ntry < tryMax && ne > TOL
                  e1 = -(obj.X(1:3,4,1) - X(1:3,4,1))*enable_contact(1);
                  e2 = -(obj.X(1:3,4,2) - X(1:3,4,2))*enable_contact(2);
                error = [e1(1:3); e2(1:3)];
                % Jpinvi = obj.jacobian_inv(J_local, lambda_damping);
                % TODO. To add exploration of null space
                % P = eye(8) - Jpinvi*J_local;
                qp = pinv(obj.J(1:6,:))*error;
                q_new = obj.q + qp*integration_step;
                obj.set_config(q_new);
                ntry = ntry+1;
                ne = norm(error);
            end
        end
        function pInvJ = jacobian_inv(~,Ji, lambda)
            if ~exist('lambda', 'var')
                lambda = .01; % damping factor
            end
            if ~exist('epsilon', 'var')
                epsilon = 1e-3; % damping factor
            end
            [U,S,V] = svd(Ji);
            for j=1:size(S,1)
                if S(j,j) < epsilon
                    S(j,j) = (S(j,j))/(S(j,j)^2+lambda^2);
                end
            end
            S = S.';
            iJJt = V*S*U.';
            pInvJ = iJJt;
        end
        function q = get_starting_config(~, cp, n)
            t = 0.5;
            nc = n(2,:)*t + n(1,:)*(1-t);
            if ( norm(nc) < 0.001)
            % if ( norm(nc) < 0.001 || isequal(n(1,:),n(2,:)))
                x_rand = rand(1,3);
                xtmp = n(2,:)-x_rand/(x_rand*n(2,:)');
                nc = xtmp/norm(xtmp);
            end
            nc = nc/norm(nc);
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
        function e = diff(~, T1, T2)
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
        function S = get_synergies(obj, n_synergies)
            if ~exist('n_synergies','var')
                n_synergies = obj.n_dof;
            end
            S = obj.S(:,1:n_synergies);
        end
    end
end