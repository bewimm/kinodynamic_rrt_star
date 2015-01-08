function [ ok ] = is_input_free( input, input_limits, time_range )
%IS_INPUT_FREE Summary of this function goes here
%   Detailed explanation goes here

ok = true;
resolution = 20;

if isa(input,'sym')

    dt = time_range(2)-time_range(1);
    r = [time_range(1):dt/resolution:time_range(2)];

    s = eval(subs(input,r));

    for ii=1:size(input_limits, 1)
        if sum(s(ii,:)<input_limits(ii, 1)) > 1 || sum(s(ii,:)>input_limits(ii, 2)) > 1
            %if ~isAlways(state(ii) >= state_limits(ii, 1)) || ~isAlways(state(ii) <= state_limits(ii, 2))
            ok = false;
            return;
        end
    end
elseif isa(input, 'function_handle')

    dt = time_range(2)-time_range(1);
    r = [time_range(1):dt/resolution:time_range(2)];

    for jj=1:length(r)
        s = input(r(jj));
        for ii=1:size(input_limits, 1)
            if s(ii) < input_limits(ii, 1) || s(ii) > input_limits(ii, 2)
                ok = false;
                return;
            end
        end
    end
else
    for ii=1:size(input_limits, 1)
        if input(ii) < input_limits(ii, 1) || input(ii) > input_limits(ii, 2)
            ok = false;
            return;
        end
    end
end

end

