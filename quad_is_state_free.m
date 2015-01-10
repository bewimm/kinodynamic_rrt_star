function [ ok ] = quad_is_state_free( state, state_limits, obstacles, radius, time_range )
%IS_STATE_FREE returns true if the given state is valid
% - state is the 10 dimensional state vector
% - state_limits limits for the state variables
% - obstacles is an n by 6 matrix where each row contains one corner and
% the distance to the other
% - quad_dim contains the size of the quadcopter bounding-box

ok = true;
max_dist = .1;

if isa(state,'sym')

    %dt = time_range(2)-time_range(1);
    r = [time_range(1):max_dist:time_range(2)];

    s = eval(subs(state,r));

    for ii=1:size(state_limits, 1)
        if sum(s(ii,:)<state_limits(ii, 1)) > 0 || sum(s(ii,:)>state_limits(ii, 2)) > 0
            %if ~isAlways(state(ii) >= state_limits(ii, 1)) || ~isAlways(state(ii) <= state_limits(ii, 2))
            ok = false;
            return;
        end
        if collides(obstacles, radius, s)
            ok = false;
            return;
        end
    end
elseif isa(state, 'function_handle')

    %dt = time_range(2)-time_range(1);
    r = [time_range(1):max_dist:time_range(2)];

    for jj=1:length(r)
        s = state(r(jj));
        for ii=1:size(state_limits, 1)
            if s(ii) < state_limits(ii, 1) || s(ii) > state_limits(ii, 2)
                ok = false;
                return;
            end
        end
        if collides(obstacles, radius, s)
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
    if collides(obstacles, radius, state)
        ok = false;
        return;
    end
end

end


function [coll] = collides(obstacles, radius, s)

n_obs = size(obstacles, 1);
coll = false;
for ii=1:n_obs
    
    obs = obstacles(ii,:)';
    obs(1:3) = obs(1:3)-obs(4:6)/2;
    
    c_min_bl = s(1:3)-obs(1:3);
    c_min_tr = s(1:3)-(obs(1:3)+obs(4:6));
    
    if s(1)>obs(1) && s(1)<obs(1)+obs(4) && s(2)>obs(2) && s(2)<obs(2)+obs(5) && s(3)>obs(3) && s(3)<obs(3)+obs(6)
        coll = true;
        return;
    end
     
    closest = min(max(s(1:3),obs(1:3)),obs(1:3)+obs(4:6));
    d = s(1:3)-closest;
    if sum(d.^2) < radius^2
        coll = true;
        return;
    end
    
    
end
            
end

