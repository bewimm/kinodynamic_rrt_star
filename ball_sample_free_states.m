function [ state ] = ball_sample_free_states( state_limits, obstacles, radius )

not_done = true;
while not_done

    state = rand(size(state_limits,1), 1);
    state = state.*(state_limits(:,2)-state_limits(:,1))+state_limits(:,1);

    not_done = ~ball_is_state_free(state, state_limits, obstacles, radius);

end


end
