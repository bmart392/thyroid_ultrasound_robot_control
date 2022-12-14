from numpy import sign

# calculate a resulting control input for a given set of gains and errors
def pd_controller(k_p, k_d, error, error_dot, minimum_control_input, maximum_control_input):
    unbounded_control_input =  k_p * error + k_d * error_dot

    control_input = unbounded_control_input
    if abs(unbounded_control_input) > maximum_control_input:
        control_input = maximum_control_input * sign(unbounded_control_input)
    if abs(unbounded_control_input) < minimum_control_input:
        control_input = minimum_control_input * sign(unbounded_control_input)
    return control_input

# calculate average error dot over error history
def calculate_error(reference_value, error_history):
    error = reference_value - error_history[0][1]
    net_error_sum = 0
    net_time_change = 0
    number_of_elements_summed = len(error_history) - 1
    for index in range(number_of_elements_summed):
        net_error_sum = net_error_sum + reference_value * (error_history[index][1] - error_history[index + 1][1])
        net_time_change = net_time_change + (error_history[index][0] - error_history[index + 1][0])
    if net_time_change == 0:
        error_dot = 0
    else:
        error_dot = net_error_sum/net_time_change
    return error, error_dot