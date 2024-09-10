% Initialize variables
filtered_value = 0.0;  % Initial value (e.g., initial speed or brightness)
target_value = 100.0;  % Desired target value (e.g., maximum speed or brightness)
alpha = 0.1;  % Smoothing factor (0 < alpha ≤ 1)
tolerance = 0.01;  % Tolerance for stopping the iteration

% Initialize array to store values
filtered_values = [];  % Array to store filtered values over time
iteration = 0;  % Initialize iteration counter

% Iterate until the value reaches close to the target
while abs(target_value - filtered_value) > tolerance
    % Apply exponential smoothing
    filtered_value = alpha * target_value + (1 - alpha) * filtered_value;
    
    % Store the filtered value for plotting
    filtered_values = [filtered_values, filtered_value];  % Append the new value to the array
    
    % Increment iteration count
    iteration = iteration + 1;
end

% Plot the results
figure; % Open a new figure window
plot(1:iteration, filtered_values, '-o', 'LineWidth', 1.5);
xlabel('Iteration');
ylabel('Filtered Value');
title('Exponential Smoothing Filter Output');
grid on;
