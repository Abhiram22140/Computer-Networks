% LEACH Protocol Simulation with Visualization and Performance Metrics in MATLAB
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

% Cluster Head Probability
p = 0.05;

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
    S(i).type = 'N';                % 'N' for Normal Node, 'C' for Cluster Head
    S(i).G = 0;                     % Counter for cluster head election
    S(i).cluster = 0;               % Cluster head ID
end

% Record of Alive Nodes
alive_nodes = n;
dead_nodes = 0;

% Metrics for Analysis
alive_history = zeros(1, max_rounds);
energy_history = zeros(1, max_rounds);
packets_to_BS = zeros(1, max_rounds);
packets_to_CH = zeros(1, max_rounds);
num_cluster_heads = zeros(1, max_rounds);

%% Simulation over Rounds

for r = 1:max_rounds
    % Reset Cluster Heads for the Round
    cluster_heads = [];
    packets_to_CH_count = 0;
    packets_to_BS_count = 0;

    % Election of Cluster Heads
    for i = 1:n
        if S(i).E > 0
            if S(i).G <= 0
                T = p / (1 - p * mod(r - 1, round(1 / p)));
                if rand <= T
                    S(i).type = 'C';
                    S(i).G = round(1 / p) - 1;
                    cluster_heads = [cluster_heads, i];
                end
            else
                S(i).G = S(i).G - 1;
            end
        else
            S(i).type = 'D';
        end
    end

    % If no cluster head elected, choose one randomly
    if isempty(cluster_heads)
        alive_nodes_idx = find([S.E] > 0);
        temp = randi(length(alive_nodes_idx));
        S(alive_nodes_idx(temp)).type = 'C';
        cluster_heads = [cluster_heads, alive_nodes_idx(temp)];
    end

    % Record number of cluster heads
    num_cluster_heads(r) = length(cluster_heads);

    % Cluster Formation
    for i = 1:n
        if strcmp(S(i).type, 'N') && S(i).E > 0
            min_dis = inf;
            min_dis_cluster = 0;
            for c = cluster_heads
                temp = sqrt((S(i).x - S(c).x)^2 + (S(i).y - S(c).y)^2);
                if temp < min_dis
                    min_dis = temp;
                    min_dis_cluster = c;
                end
            end
            S(i).cluster = min_dis_cluster;
            % Energy consumption for transmitting join request
            distance = min_dis;
            if distance > 0
                if distance < d0
                    S(i).E = S(i).E - (E_elec * k + E_fs * k * distance^2);
                else
                    S(i).E = S(i).E - (E_elec * k + E_mp * k * distance^4);
                end
                packets_to_CH_count = packets_to_CH_count + 1;
            end
        end
    end

    % Data Transmission from Nodes to Cluster Heads
    for i = 1:n
        if strcmp(S(i).type, 'N') && S(i).E > 0
            ch = S(i).cluster;
            if ch > 0 && S(ch).E > 0
                distance = sqrt((S(i).x - S(ch).x)^2 + (S(i).y - S(ch).y)^2);
                if distance > 0
                    if distance < d0
                        S(i).E = S(i).E - (E_elec * k + E_fs * k * distance^2);
                    else
                        S(i).E = S(i).E - (E_elec * k + E_mp * k * distance^4);
                    end
                    % Cluster head receives data
                    S(ch).E = S(ch).E - (E_elec * k + E_DA * k);
                    packets_to_CH_count = packets_to_CH_count + 1;
                end
            end
        end
    end

    % Data Transmission from Cluster Heads to Base Station
    for ch = cluster_heads
        if S(ch).E > 0
            distance = sqrt((S(ch).x - sink.x)^2 + (S(ch).y - sink.y)^2);
            if distance > 0
                if distance < d0
                    S(ch).E = S(ch).E - (E_elec * k + E_fs * k * distance^2);
                else
                    S(ch).E = S(ch).E - (E_elec * k + E_mp * k * distance^4);
                end
                packets_to_BS_count = packets_to_BS_count + 1;
            end
        end
    end

    % Reset node type to Normal for next round
    for i = cluster_heads
        if S(i).E > 0
            S(i).type = 'N';
        end
    end

    % Update Energy and Status of Nodes
    dead_this_round = 0;
    for i = 1:n
        if S(i).E <= 0 && ~strcmp(S(i).type, 'D')
            S(i).E = 0;
            S(i).type = 'D'; % Dead
            dead_nodes = dead_nodes + 1;
            alive_nodes = alive_nodes - 1;
            dead_this_round = dead_this_round + 1;
        end
    end

    % Record Metrics
    alive_history(r) = alive_nodes;
    energy_history(r) = sum([S.E]);
    packets_to_CH(r) = packets_to_CH_count;
    packets_to_BS(r) = packets_to_BS_count;

    %% Display Performance Metrics in Console

    total_energy = sum([S.E]); % Total remaining energy in the network
    if alive_nodes > 0
        average_energy = total_energy / alive_nodes; % Average energy per alive node
    else
        average_energy = 0;
    end

    fprintf('Round %d:\n', r);
    fprintf('  Alive Nodes            : %d\n', alive_nodes);
    fprintf('  Dead Nodes             : %d\n', dead_nodes);
    fprintf('  Total Energy (J)       : %.4f\n', total_energy);
    fprintf('  Average Energy (J)     : %.4f\n', average_energy);
    fprintf('  Packets to BS          : %d\n', packets_to_BS_count);
    fprintf('  Packets to CH          : %d\n', packets_to_CH_count);
    fprintf('  Number of Cluster Heads: %d\n', num_cluster_heads(r));
    fprintf('-----------------------------\n');

    %% Visualization of Data Aggregation

    figure(5);
    clf;
    hold on;
    grid on;
    axis([0 xm 0 ym]);
    title(sprintf('LEACH Protocol - Round %d', r));
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

    % Highlight Cluster Heads
    for i = cluster_heads
        plot(S(i).x, S(i).y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    end

    % Plot Base Station
    plot(sink.x, sink.y, 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');

    % Draw lines from member nodes to their cluster heads
    for i = 1:n
        if strcmp(S(i).type, 'N') && S(i).E > 0 && S(i).cluster > 0
            ch = S(i).cluster;
            if S(ch).E > 0
                line([S(i).x, S(ch).x], [S(i).y, S(ch).y], 'Color', 'c', 'LineStyle', '--');
            end
        end
    end

    % Draw lines from cluster heads to base station
    for ch = cluster_heads
        if S(ch).E > 0
            line([S(ch).x, sink.x], [S(ch).y, sink.y], 'Color', 'm', 'LineWidth', 2);
        end
    end

    legend('Alive Nodes', 'Dead Nodes', 'Cluster Heads', 'Base Station', 'Location', 'bestoutside');
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

% Packets Sent to Cluster Heads Over Rounds
figure(4);
plot(1:r, packets_to_CH(1:r));
xlabel('Rounds');
ylabel('Packets Sent to Cluster Heads');
title('Traffic Load on Cluster Heads');
grid on;
