% Hybrid PEGASIS-TEEN Protocol with Round-by-Round Performance Metrics
clear all;
clc;

%% Network and Simulation Parameters

% Network Field Dimensions
xm = 100; % x-axis
ym = 100; % y-axis

% Number of Sensor Nodes
n = 100;

% Base Station Coordinates
sink.x = 50;
sink.y = 50;

% Initial Energy (J)
Eo = 0.5;

% Energy Model Parameters
E_elec = 50e-9;      % Energy for electronics (J/bit)
E_fs = 10e-12;       % Free-space model (J/bit/m^2)
E_mp = 0.0013e-12;   % Multi-path model (J/bit/m^4)
E_DA = 5e-9;         % Data aggregation (J/bit)

% Threshold Distance
d0 = sqrt(E_fs / E_mp);

% Data Packet Size
k = 4000; % in bits

% Thresholds for TEEN
hard_threshold = 55; % Hard threshold
soft_threshold = 5;  % Soft threshold

% Simulation Rounds
max_rounds = 100;

%% Node Initialization
nodes = struct();
for i = 1:n
    nodes(i).x = rand * xm;
    nodes(i).y = rand * ym;
    nodes(i).E = Eo;           % Residual Energy
    nodes(i).role = 'Normal';  % Node role: Normal, Leader, or Dead
    nodes(i).cluster = 0;      % Assigned cluster (for visualization)
    nodes(i).data_value = 0;   % Last sensed data value
    nodes(i).data_sent = false; % Flag to track data transmission
end

% Track Metrics
alive_history = zeros(1, max_rounds);
energy_history = zeros(1, max_rounds);
packets_to_BS = zeros(1, max_rounds);
packets_to_CH = zeros(1, max_rounds);
total_energy_consumed = zeros(1, max_rounds);

%% Simulation Loop
for round = 1:max_rounds
    % Step 1: Chain Formation
    alive_nodes = find([nodes.E] > 0);
    num_alive = length(alive_nodes);
    
    if num_alive == 0
        break; % End simulation if all nodes are dead
    end
    
    chain = [];
    remaining_nodes = alive_nodes;
    [~, start_node] = min(arrayfun(@(i) sqrt((nodes(i).x - sink.x)^2 + (nodes(i).y - sink.y)^2), remaining_nodes));
    current_node = remaining_nodes(start_node);
    chain = [chain, current_node];
    remaining_nodes(start_node) = [];
    
    while ~isempty(remaining_nodes)
        distances = arrayfun(@(i) sqrt((nodes(i).x - nodes(current_node).x)^2 + (nodes(i).y - nodes(current_node).y)^2), remaining_nodes);
        [~, nearest_node] = min(distances);
        current_node = remaining_nodes(nearest_node);
        chain = [chain, current_node];
        remaining_nodes(nearest_node) = [];
    end
    
    % Step 2: Leader Election
    leader_idx = chain(1); % Choose the first node in the chain
    max_energy = nodes(leader_idx).E;
    for i = chain
        if nodes(i).E > max_energy
            leader_idx = i;
            max_energy = nodes(i).E;
        end
    end
    nodes(leader_idx).role = 'Leader';
    
    % Assign cluster memberships for visualization
    for i = chain
        if i ~= leader_idx
            nodes(i).cluster = leader_idx;
        end
    end
    
    % Step 3: Data Transmission
    packets_to_CH_count = 0;
    for i = chain
        if nodes(i).E > 0 && ~strcmp(nodes(i).role, 'Leader')
            % Sense the attribute
            sensed_value = hard_threshold + rand * 20; % Simulated data
            % Check thresholds
            if sensed_value >= hard_threshold
                if abs(sensed_value - nodes(i).data_value) >= soft_threshold || ~nodes(i).data_sent
                    % Transmit to leader
                    distance = sqrt((nodes(i).x - nodes(leader_idx).x)^2 + (nodes(i).y - nodes(leader_idx).y)^2);
                    if distance < d0
                        nodes(i).E = nodes(i).E - (E_elec * k + E_fs * k * distance^2);
                    else
                        nodes(i).E = nodes(i).E - (E_elec * k + E_mp * k * distance^4);
                    end
                    nodes(i).data_value = sensed_value;
                    nodes(i).data_sent = true;
                    packets_to_CH_count = packets_to_CH_count + 1;
                end
            end
        end
    end
    
    % Leader sends data to base station
    packets_to_BS_count = 0;
    if nodes(leader_idx).E > 0
        distance_to_sink = sqrt((nodes(leader_idx).x - sink.x)^2 + (nodes(leader_idx).y - sink.y)^2);
        if distance_to_sink < d0
            nodes(leader_idx).E = nodes(leader_idx).E - (E_elec * k + E_fs * k * distance_to_sink^2);
        else
            nodes(leader_idx).E = nodes(leader_idx).E - (E_elec * k + E_mp * k * distance_to_sink^4);
        end
        packets_to_BS_count = packets_to_BS_count + 1;
    end
    
    % Update Node Status
    for i = 1:n
        if nodes(i).E <= 0
            nodes(i).E = 0;
            nodes(i).role = 'Dead';
        end
    end
    
    % Record Metrics
    alive_history(round) = num_alive;
    energy_history(round) = sum([nodes.E]);
    packets_to_CH(round) = packets_to_CH_count;
    packets_to_BS(round) = packets_to_BS_count;
    total_energy_consumed(round) = n * Eo - sum([nodes.E]);
    
    %% Print Metrics for the Current Round
    fprintf('Round %d:\n', round);
    fprintf('  Alive Nodes                  : %d\n', num_alive);
    fprintf('  Total Energy Remaining (J)   : %.4f\n', sum([nodes.E]));
    fprintf('  Total Energy Consumed (J)    : %.4f\n', total_energy_consumed(round));
    fprintf('  Average Energy Consumed (J)  : %.4f\n', total_energy_consumed(round) / n);
    fprintf('  Packets to Cluster Head      : %d\n', packets_to_CH_count);
    fprintf('  Packets to Base Station      : %d\n', packets_to_BS_count);
    fprintf('  Leader Node ID               : %d\n', leader_idx);
    fprintf('-----------------------------\n');
    
    %% Visualization
    figure(1);
    clf;
    hold on;
    axis([0 xm 0 ym]);
    title(sprintf('Hybrid PEGASIS-TEEN Protocol - Round %d', round));
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    
    % Plot all nodes
    for i = 1:n
        if nodes(i).E > 0
            plot(nodes(i).x, nodes(i).y, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
        else
            plot(nodes(i).x, nodes(i).y, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
        end
    end
    
    % Highlight leader
    plot(nodes(leader_idx).x, nodes(leader_idx).y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    
    % Draw lines from member nodes to their leader
    for i = 1:n
        if nodes(i).E > 0 && nodes(i).cluster > 0
            line([nodes(i).x, nodes(nodes(i).cluster).x], [nodes(i).y, nodes(nodes(i).cluster).y], 'Color', 'c', 'LineStyle', '--');
        end
    end
    
    % Plot sink
    plot(sink.x, sink.y, 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    
    legend('Alive Nodes', 'Dead Nodes', 'Leader Node', 'Cluster Connections', 'Sink', 'Location', 'bestoutside');
    hold off;
    pause(0.1); % Adjust pause duration for visualization speed
end

%% Final Metrics
initial_energy = n * Eo;
final_energy = sum([nodes.E]);
total_energy_consumed = initial_energy - final_energy;

fprintf('\nSimulation Completed after %d rounds.\n', round);
fprintf('Final Total Energy Consumed (J): %.4f\n', total_energy_consumed);
fprintf('Final Average Energy Consumed per Node (J): %.4f\n', total_energy_consumed / n);

% Final Plots
figure(2);
plot(1:round, alive_history(1:round));
xlabel('Rounds');
ylabel('Number of Alive Nodes');
title('Network Lifetime');
grid on;

figure(3);
plot(1:round, energy_history(1:round));
xlabel('Rounds');
ylabel('Total Energy of Network (J)');
title('Energy Consumption Over Time');
grid on;
