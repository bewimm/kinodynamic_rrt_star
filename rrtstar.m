classdef rrtstar
    %RRTSTAR Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = private)
        A_ = [];
        B_ = [];
        c_ = [];
        R_ = [];
        best_arrival = [];
        t = sym('t', 'positive');
        x = sym('x', 'positive'); %used for integration
        t_s = sym('t_s', 'real'); %optimal arrival time

        x0 = [];
        x1 = [];

        states_eq = [];
        control_eq = [];
        cost_eq = [];

        eval_arrival_internal;
        eval_cost_internal;
        eval_states_internal;
        eval_control_internal;

    end

    methods
        function obj = rrtstar(A, B, c, R)

            if rank(ctrb(A,B)) ~= size(A,1)
                disp('System is not controllable - aborting');
                return;
            end

            obj.A_ = A;
            obj.B_ = B;
            obj.c_ = c;
            obj.R_ = R;

            state_dims = size(A,1);
            obj.x0 = sym('x0',[state_dims,1]);
            obj.x0 = sym(obj.x0, 'real');
            obj.x1 = sym('x1',[state_dims,1]);
            obj.x1 = sym(obj.x1, 'real');

            [obj.best_arrival, obj.states_eq, obj.control_eq, obj.cost_eq] = obj.calc_equations(obj.A_,obj.B_,obj.R_,obj.c_);


            %%calculate the factors for the resulting polynomial explicitly
            p1 = [];
            it = 0;
            while it < 20 && length(p1) <= 1 %just so mupad knows it's a polynomial

                p1 = feval(symengine,'coeff',simplifyFraction(obj.best_arrival*obj.t^it), obj.t, 'All');
                it = it+1;
            end
            if it > 20
                disp('either the result is not a polynomial or the degree is to high');
            end

            p([obj.x0', obj.x1']) = fliplr(p1);
            obj.eval_arrival_internal = matlabFunction(p); %matlabFunction(p, 'file', 'arrival_time.m'); %to write to file

            obj.eval_cost_internal = matlabFunction(obj.cost_eq);
            obj.eval_states_internal = matlabFunction(obj.states_eq);
            obj.eval_control_internal = matlabFunction(obj.control_eq);

        end

        function [time] = evaluate_arrival_time(obj, x0_, x1_)
            %arr = subs(obj.best_arrival, obj.x0, x0_);
            %arr = subs(arr, obj.x1, x1_);
            %time_a = min(eval(solve(arr, 'Real', true))); %symbolical version

            in = num2cell([x0_', x1_']);
            time = roots(obj.eval_arrival_internal(in{:}));
            time = time(imag(time)==0);
            time = min(time(time>=0));

        end

        function [cost, time] = evaluate_cost(obj, x0_, x1_)

            time = obj.evaluate_arrival_time(x0_, x1_);

            %c = subs(obj.cost_eq, obj.x0, x0_);
            %c = subs(c, obj.x1, x1_);
            %c = subs(c, obj.t_s, time);
            %cost = eval(c);

            in = num2cell([time, x0_', x1_']);
            cost = obj.eval_cost_internal(in{:});
        end

        function [states, inputs] = evaluate_states_and_inputs(obj, x0_, x1_)

            time = obj.evaluate_arrival_time(x0_, x1_);

            %s = subs(obj.states_eq, obj.x0, x0_);
            %s = subs(s, obj.x1, x1_);
            %states = subs(s, obj.t_s, time);

            %c = subs(obj.control_eq, obj.x0, x0_);
            %c = subs(c, obj.x1, x1_);
            %inputs = subs(c, obj.t_s, time);

            in = num2cell([time, x0_', x1_']);
            states = @(t)obj.eval_states_internal(t, in{:});
            inputs = @(t)obj.eval_control_internal(t, in{:});

        end


        function [T, parents] = run(obj, sample_free_state, is_state_free, is_input_free, start, goal, display)
            T = [start];
            costs = [0];
            parents = [-1];

            max_it = 1000;
            r = 40;

            goal_cost = inf;
            goal_parent = 0;

            for it=1:max_it

                disp(['iteration ',num2str(it), ' of ', num2str(max_it)]);
                sample_ok = false;
                while ~sample_ok

                    x_i = sample_free_state();%sample_free_states(state_limits, obstacles, quad_dim);
                    min_idx = 1;
                    min_cost = inf;
                    for jj=1:size(T,2)
                        [cost, time] = evaluate_cost(obj, T(:,jj),x_i);
                        if cost < r && costs(jj)+cost < min_cost
                            [states, u] = evaluate_states_and_inputs(obj, T(:,jj),x_i);
                            if  is_state_free(states,[0,time]) && is_input_free(u,[0,time])
                                sample_ok = true;
                                min_idx = jj;
                                min_cost = costs(jj)+cost;
                            end
                        end
                    end

                end

                parents = [parents, min_idx];
                costs = [costs, min_cost];

                for jj=1:size(T,2)
                    [cost, time] = evaluate_cost(obj, x_i,T(:,jj));
                    if cost < r && costs(end)+cost < costs(jj)
                        [states, u] = evaluate_states_and_inputs(obj, x_i,T(:,jj));
                        if is_state_free(states,[0,time]) && is_input_free(u,[0,time])
                            costs(jj) = costs(end)+cost;
                            parents(jj) = size(parents,2);
                        end
                    end
                end

                [cost, time] = evaluate_cost(obj, x_i, goal);
                if costs(end)+cost < goal_cost
                    [states, u] = evaluate_states_and_inputs(obj, x_i,goal);
                    if is_state_free(states,[0,time]) && is_input_free(u,[0,time])
                        goal_cost = costs(end)+cost;
                        goal_parent = size(T,2)+1;
                    end
                end

                T = [T,x_i];

                display(obj, T, parents, goal_cost, goal_parent);

            end
        end


    end

    methods (Access = protected)

        function [tau_star, states, control, cost] = calc_equations(obj, A, B, R, c)
            %the solution of the equation 'tau_star = 0' gives the best arrival time
            %states, control, and cost depend on t_s which is the solution from above
            state_dims = size(A,1);

            G = int(expm(A*(obj.t-obj.x))*B/R*B'*expm(A'*(obj.t-obj.x)), obj.x, 0, obj.t);
            x_bar = expm(A*obj.t)*obj.x0+int(expm(A*(obj.t-obj.x))*c, obj.x, 0, obj.t);

            d = G\(obj.x1-x_bar);
            tau_star = 1-2*(A*obj.x1+c)'*d-d'*B/R*B'*d;


            solution = expm([A, B/R*B';zeros(state_dims), -A']*(obj.t-obj.t_s))*[obj.x1;subs(d,obj.t,obj.t_s)]+ ...
                int(expm([A, B/R*B';zeros(state_dims), -A']*(obj.t-obj.x))*[c; zeros(state_dims,1)],obj.x,obj.t_s,obj.t);

            control = R\B'*solution(state_dims+1:2*state_dims,:);
            states = solution(1:state_dims);

            cost = int(1+control'*R*control, obj.t, 0, obj.t_s);

        end

    end

end

