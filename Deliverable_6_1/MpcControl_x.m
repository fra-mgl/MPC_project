classdef MpcControl_x < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];
            
            % state constraints
            F = [0 1 0 0 ; 0 -1 0 0];
            f = [0.17; 0.17];
            
            % input constraints
            G = [1 -1]';
            g = [0.26; 0.26];

            % Q = diag([100 100 1000 10000]);
            % R = 0.1;
            Q = diag([10 10 10 60]);
            R = 0.00001;
            [~, P,~] = dlqr(mpc.A, mpc.B, Q, R); % optimal LQR controller
    

            % ----- SLACK VARIABLES ----- %
            % epsilon same dimension as state constraints
            EPS = sdpvar(2, N);
            S = diag([100 100]);
            ro = @(eps) eps'*S*eps;

            % ----- ADD CONSTRAINTS ----- %

            % add constraints and objective to YALMIN optimization solver
            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)) + (G*U(:,1) <= g);
            obj = (U(:,1)-u_ref)'*R*((U(:,1)-u_ref));
            for i = 2:N-1
                con = con + (X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i));
                con = con + (F*X(:,i) <= f + EPS(:,i)) + (G*U(:,i) <= g);
                obj = obj + (X(:,i) - x_ref)'*Q*(X(:,i) - x_ref) + (U(:,i) - u_ref)'*R*(U(:,i) - u_ref) + ro(EPS(:,i)) + norm(EPS(:,i),1);
                %obj = obj + X(:,i)'*Q*X(:,i) + (U(:,i) - U(:,i-1))'*R*(U(:,i) - U(:,i-1));
            end
           
            obj = obj + (X(:,N) - x_ref)'*P*(X(:,N) - x_ref) + ro(EPS(:,N)) + norm(EPS(:,N),1);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = 0;
            con = [xs == 0, us == 0];

            % input constraints
            G = [1 -1]';
            g = [0.26; 0.26];
            
            % state constraints
            F = [0 1 0 0 ; 0 -1 0 0];
            f = [0.17; 0.17];
            
            obj = us^2;
            con = (eye(4)-mpc.A)*xs-mpc.B*us == 0;
            con = [con, mpc.C*xs==ref];
            con = [con, G*us<=g];
            con = [con, F*xs<=f];

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
