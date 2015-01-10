function [ state ] = quad_sample_free_states( state_limits, obstacles, quad_dim )

not_done = true;
while not_done

    state = rand(size(state_limits,1), 1);
    state = state.*(state_limits(:,2)-state_limits(:,1))+state_limits(:,1);

    not_done = ~quad_is_state_free(state, state_limits, obstacles, quad_dim);

end


end

