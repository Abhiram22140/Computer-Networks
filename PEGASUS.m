% PEGASIS Protocol Simulation with Visualization and Performance Metrics in MATLAB
% Energy-Efficient Data Aggregation for Wireless Sensor Networks

% Clear the workspace and command window
clear all;
clc;

%% Network and Simulation Parameters

% Network Field Dimensions (in meters)
xm = 100; % x-axis length
ym = 100; % y-axis length

% Number of Sensor Nodes
n = 100;

% Sink (Base Station) Coordinates
sink.x = 50; % x-coordinate of sink
sink.y = 50; % y-coordinate of sink

% Number of Rounds for Simulation
max_rounds = 100;

% Initial Energy of Nodes (in Joules)
Eo = 0.5;

% Energy Model Parameters
E_elec = 50e-9;      % Energy dissipation of electronics (J/bit)
E_fs = 10e-12;       % Free space model amplifier energy (J/bit/m^2)
E_mp = 0.0013e-12;   % Multi-path model amplifier energy (J/bit/m^4)
E_DA = 5e-9;         % Data aggregation energy (J/bit)

% Data Packet Size
k = 4000; % Packet size in bits

% Threshold Distance
d0 = sqrt(E_fs / E_mp);

% Initialize Random Number Generator for Reproducibility
rng(1);

%% Initialize Sensor Nodes

% Structure to hold node information
S = struct();

for i = 1:n
    S(i).id = i;                    % Node ID
    S(i).x = rand * xm;             % x-coordinate
    S(i).y = rand * ym;             % y-coordinate
    S(i).E = Eo;                    % Initial Energy
    S(i).neighbor = [];             % Neighbor in the chain
    S(i).type = 'N';                % 'N' for Normal Node
end

% Record of Alive Nodes
alive_nodes = n;
dead_nodes = 0;

% Metrics for Analysis
alive_history = zeros(1, max_rounds);
energy_history = zeros(1, max_rounds);
packets_to_BS = zeros(1, max_rounds);
num_transmissions = zeros(1, max_rounds);

%% Simulation over Rounds

for r = 1:max_rounds
    % Reset node neighbors for the round
    for i = 1:n
        S(i).neighbor = [];
    end

    % Construct the Chain using Greedy Algorithm

    % Get indices of alive nodes
    alive_indices = find([S.E] > 0);
    num_alive = length(alive_indices);

    if num_alive == 0
        fprintf('All nodes are dead at round %d.\n', r);
        break; % All nodes are dead
    end

    % Positions of alive nodes
    positions = [[S(alive_indices).x]', [S(alive_indices).y]'];

    % Start with the node closest to the sink
    distances_to_sink = sqrt((positions(:,1) - sink.x).^2 + (positions(:,2) - sink.y).^2);
    [~, min_index] = min(distances_to_sink);
    start_node = alive_indices(min_index);

    % Initialize chain
    chain = start_node;
    visited = false(n,1);
    visited(start_node) = true;

    for i = 1:num_alive - 1
        last_node = chain(end);
        % Find the closest unvisited alive node
        min_dist = inf;
        min_node = -1;
        for j = alive_indices
            if ~visited(j)
                dist = sqrt((S(last_node).x - S(j).x)^2 + (S(last_node).y - S(j).y)^2);
                if dist < min_dist
                    min_dist = dist;
                    min_node = j;
                end
            end
        end
        if min_node == -1
            break;
        end
        chain = [chain, min_node];
        visited(min_node) = true;
        S(last_node).neighbor = min_node;
    end

    % Set neighbor of last node to empty
    S(chain(end)).neighbor = [];

    % Determine Leader for the Round (nodes take turns)
    leader_index = mod(r - 1, length(chain)) + 1;
    leader_node = chain(leader_index);

    % Data Transmission Along the Chain
    packets_to_BS_count = 0;
    transmissions = 0;

    % From end of chain to leader
    for i = length(chain):-1:1
        node = chain(i);
        if S(node).E > 0
            if node ~= leader_node
                % Ensure that i+1 does not exceed the length of the chain
                if i < length(chain)
                    neighbor = chain(i + 1);
                    if S(neighbor).E > 0
                        distance = sqrt((S(node).x - S(neighbor).x)^2 + (S(node).y - S(neighbor).y)^2);
                        if distance < d0
                            S(node).E = S(node).E - (E_elec * k + E_fs * k * distance^2);
                        else
                            S(node).E = S(node).E - (E_elec * k + E_mp * k * distance^4);
                        end
                        % Receiving node consumes energy
                        S(neighbor).E = S(neighbor).E - (E_elec * k + E_DA * k);
                        transmissions = transmissions + 1;
                    end
                end
            else
                % Leader node sends data to the sink
                distance = sqrt((S(node).x - sink.x)^2 + (S(node).y - sink.y)^2);
                if distance < d0
                    S(node).E = S(node).E - (E_elec * k + E_fs * k * distance^2);
                else
                    S(node).E = S(node).E - (E_elec * k + E_mp * k * distance^4);
                end
                packets_to_BS_count = packets_to_BS_count + 1;
                transmissions = transmissions + 1;
            end
        end
    end

    % Update Energy and Status of Nodes
    dead_this_round = 0;
    alive_nodes = 0;
    for i = 1:n
        if S(i).E <= 0 && ~strcmp(S(i).type, 'D')
            S(i).E = 0;
            S(i).type = 'D'; % Dead
            dead_nodes = dead_nodes + 1;
            dead_this_round = dead_this_round + 1;
        end
        if S(i).E > 0
            alive_nodes = alive_nodes + 1;
        end
    end

    % Record Metrics
    alive_history(r) = alive_nodes;
    energy_history(r) = sum([S.E]);
    packets_to_BS(r) = packets_to_BS_count;
    num_transmissions(r) = transmissions;

    %% Display Performance Metrics in Console

    total_energy = sum([S.E]); % Total remaining energy in the network
    if alive_nodes > 0
        average_energy = total_energy / alive_nodes; % Average energy per alive node
    else
        average_energy = 0;
    end

    fprintf('Round %d:\n', r);
    fprintf('  Alive Nodes       : %d\n', alive_nodes);
    fprintf('  Dead Nodes        : %d\n', dead_nodes);
    fprintf('  Total Energy (J)  : %.4f\n', total_energy);
    fprintf('  Average Energy (J): %.4f\n', average_energy);
    fprintf('  Packets to BS     : %d\n', packets_to_BS_count);
    fprintf('  Number of Transmissions: %d\n', transmissions);
    fprintf('-----------------------------\n');

    %% Visualization of Data Aggregation

    figure(5);
    clf;
    hold on;
    grid on;
    axis([0 xm 0 ym]);
    title(sprintf('PEGASIS Protocol - Round %d', r));
    xlabel('X Position (m)');
    ylabel('Y Position (m)');

    % Plot all nodes
    for i = 1:n
        if S(i).E > 0
            plot(S(i).x, S(i).y, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
        else
            plot(S(i).x, S(i).y, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
        end
    end

    % Plot the chain
    for i = 1:length(chain)-1
        node = chain(i);
        next_node = chain(i+1);
        if S(node).E > 0 && S(next_node).E > 0
            line([S(node).x, S(next_node).x], [S(node).y, S(next_node).y], 'Color', 'c', 'LineStyle', '--');
        end
    end

    % Highlight Leader Node
    if S(leader_node).E > 0
        plot(S(leader_node).x, S(leader_node).y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        % Draw line from leader node to base station
        line([S(leader_node).x, sink.x], [S(leader_node).y, sink.y], 'Color', 'm', 'LineWidth', 2);
    end

    % Plot Base Station
    plot(sink.x, sink.y, 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');

    legend('Alive Nodes', 'Dead Nodes', 'Chain Links', 'Leader Node', 'Base Station', 'Location', 'bestoutside');
    hold off;
    drawnow;
    pause(0.1); % Pause to visualize each round; adjust as needed

    % Terminate if All Nodes are Dead
    if alive_nodes == 0
        fprintf('All nodes are dead at round %d.\n', r);
        break;
    end
end

%% Calculate and Display Average Energy Consumed

initial_total_energy = n * Eo;
final_total_energy = sum([S.E]);
total_energy_consumed = initial_total_energy - final_total_energy;
average_energy_consumed = total_energy_consumed / n;

fprintf('\nSimulation Completed after %d rounds.\n', r);
fprintf('Total Energy Consumed (J)            : %.4f\n', total_energy_consumed);
fprintf('Average Energy Consumed per Node (J) : %.4f\n', average_energy_consumed);

%% Plot Overall Results

% Alive Nodes Over Rounds
figure(1);
plot(1:r, alive_history(1:r));
xlabel('Rounds');
ylabel('Number of Alive Nodes');
title('Network Lifetime');
grid on;

% Energy Consumption Over Rounds
figure(2);
plot(1:r, energy_history(1:r));
xlabel('Rounds');
ylabel('Total Energy of Network (J)');
title('Energy Consumption Over Time');
grid on;

% Packets Sent to Base Station Over Rounds
figure(3);
plot(1:r, packets_to_BS(1:r));
xlabel('Rounds');
ylabel('Packets Sent to Base Station');
title('Network Throughput');
grid on;

% Number of Transmissions Over Rounds
figure(4);
plot(1:r, num_transmissions(1:r));
xlabel('Rounds');
ylabel('Number of Transmissions');
title('Communication Load Over Time');
grid on;
