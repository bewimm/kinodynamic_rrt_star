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
        end
        
        function [time] = evaluate_arrival_time(obj, x0_, x1_)
            arr = subs(obj.best_arrival, obj.x0, x0_);
            arr = subs(arr, obj.x1, x1_);
            %factor = obj.t^4;
            factor = obj.t^8; %shift to be able to turn it into polynomial
            time = roots(sym2poly(arr*factor));
            time = time(imag(time)==0);
            time = min(time(time>=0));
            %time = min(eval(solve(arr, 'Real', true))); %analytical version
        end
        
        function [cost, time] = evaluate_cost(obj, x0_, x1_)
            
            time = obj.evaluate_arrival_time(x0_, x1_);
            
            c = subs(obj.cost_eq, obj.x0, x0_);
            c = subs(c, obj.x1, x1_);
            c = subs(c, obj.t_s, time);
            
            cost = eval(c);            
        end
        
        function [states, inputs] = evaluate_states_and_inputs(obj, x0_, x1_)
            
            time = obj.evaluate_arrival_time(x0_, x1_);
                  
            s = subs(obj.states_eq, obj.x0, x0_);
            s = subs(s, obj.x1, x1_);
            states = subs(s, obj.t_s, time);
            
            c = subs(obj.control_eq, obj.x0, x0_);
            c = subs(c, obj.x1, x1_);
            inputs = subs(c, obj.t_s, time);
                      
        end
        
        
        function [T, parents] = run(obj, sample_free_state, is_state_free, is_input_free, start, goal)
            T = [start];
            costs = [0];
            parents = [-1];

            max_it = 100;
            
            r = 100;

            for it=1:max_it

                disp(['iteration ',num2str(it), ' of ', num2str(max_it)]);
                sample_ok = false;
                while ~sample_ok
                
                    x_i = sample_free_state();%sample_free_states(state_limits, obstacles, quad_dim);
                    min_idx = 1;
                    min_cost = 1e40;
                    for jj=1:size(T,2)
                        [cost, time] = evaluate_cost(obj, T(:,jj),x_i);
                        %assumeAlso(symvar(states)<time)
                        if costs(jj)+cost < r && costs(jj)+cost < min_cost
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

                T_goal = [T, goal];

                for jj=1:size(T_goal,2)
                    [cost, time] = evaluate_cost(obj, x_i,T_goal(:,jj));
                    if costs(jj)+cost < r && costs(end)+cost < costs(jj)
                        [states, u] = evaluate_states_and_inputs(obj, x_i,T_goal(:,jj));
                        if is_state_free(states,[0,time]) && is_input_free(u,[0,time])
                            costs(jj) = costs(end)+cost;
                            parent(jj) = size(parent,2);
                        end
                    end
                end

                T = [T,x_i];

                plot_graph(T, parents);

            end 
        end
        
        
    end
    
    methods (Access = protected)
       
        function [tau_star, states, control, cost] = calc_equations(obj, A, B, R, c)
        %the solution of the equation 'tau_star = 0' gives the best arrival
        %time
        %states, control, and cost depend on t_s which is the solution
        %from above
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

