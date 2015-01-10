function [ state ] = quad_sample_free_states(sampling_limits, state_limits, obstacles, quad_dim )

not_done = true;
while not_done
 
    state = rand(size(sampling_limits,1), 1);
    state = state.*(sampling_limits(:,2)-sampling_limits(:,1))+sampling_limits(:,1);

    not_done = ~quad_is_state_free(state, state_limits, obstacles, quad_dim);

end


end

