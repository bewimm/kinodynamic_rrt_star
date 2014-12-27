function [ ok ] = is_state_free( state, state_limits, obstacles, quad_dim, time_range )
%IS_STATE_FREE returns true if the given state is valid
% - state is the 10 dimensional state vector
% - state_limits limits for the state variables
% - obstacles is an n by 6 matrix where each row contains one corner and
% the distance to the other
% - quad_dim contains the size of the quadcopter bounding-box 

ok = true;
resolution = 20;

if isa(state,'sym')
    
    dt = time_range(2)-time_range(1);
    r = [time_range(1):dt/resolution:time_range(2)];
    
    s = eval(subs(state,r));
    
    for ii=1:size(state_limits, 1)
        if sum(s(ii,:)<state_limits(ii, 1)) > 0 || sum(s(ii,:)>state_limits(ii, 2)) > 0
        %if ~isAlways(state(ii) >= state_limits(ii, 1)) || ~isAlways(state(ii) <= state_limits(ii, 2))
            ok = false;
            return;
        end
    end 
else
    for ii=1:size(state_limits, 1)
        if state(ii) < state_limits(ii, 1) || state(ii) > state_limits(ii, 2)
            ok = false;
            return;
        end
    end 
end



max_dim = max(max(quad_dim));

n_obs = size(obstacles, 1);
for ii=1:n_obs     
    
    a = obstacles(ii,1:3);
    b = a+[obstacles(ii, 4), 0, 0];
    c = a+[0, obstacles(ii, 5), 0];
    d = a+[0, 0, obstacles(ii, 6)];
    e = a+[obstacles(ii, 4), obstacles(ii, 5), 0];
    f = a+[obstacles(ii, 4), 0, obstacles(ii, 6)];
    g = a+[0, obstacles(ii, 5), obstacles(ii, 6)];
    h = a+obstacles(ii, 4:6);
    
    p = state(1:3)';
    maxd = max_dim^2;
    
    if (p-a)*(p-a)' < maxd || (p-b)*(p-b)' < maxd || (p-c)*(p-c)' < maxd || (p-d)*(p-d)' < maxd ...
    || (p-e)*(p-e)' < maxd || (p-f)*(p-f)' < maxd || (p-g)*(p-g)' < maxd || (p-h)*(p-h)' < maxd
       ok = false;
       return
    end
    
end

end

