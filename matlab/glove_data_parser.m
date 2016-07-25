% Tactile Glove Data Parser
% implemented by Jangwon Lee
% leejang@indiana.edu

function [] = glove_data_parser(force_data_file, imu_data_file)

    disp('glove_data_parser');

    % add Kevin Murphy's HMM Matlab toolbox
    addpath(genpath('/home/leejang/lib/HMMall'));

    % parse forse data file
    force_data = parse_force_data(force_data_file);

    % force data without time
    force_data_wo_time = force_data(:,3:9);  

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % HMM based data segmentation
    % to see just first attempt

    sub_data = force_data(401:2000,:);

    % observed sequence
    seq = sub_data(:,3:9)';

    % Number of states
    Q = 3;   
    % Number of observations
    O = 7;
    % Number of data in a sequence
    T = 1600;
    % Number of sequences
    N = 1;
    % Number of mixtures
    M = 1;

    % initial guess of parameters
    prior0 = normalise(rand(Q,1));
    transmat0 = mk_stochastic(rand(Q,Q));

    [mu0, Sigma0] = mixgauss_init(Q*M, seq, 'full');
    mu0 = reshape(mu0, [O Q M]);
    Sigma0 = reshape(Sigma0, [O O Q M]);
    mixmat0 = mk_stochastic(rand(Q,M));

    [LL, prior1, transmat1, mu1, Sigma1, mixmat1] = ...
    mhmm_em(seq, prior0, transmat0, mu0, Sigma0, mixmat0, 'max_iter', 4);

    % Emission probailities
    E = mixgauss_prob(seq, mu1, Sigma1, mixmat1);

    % Most probable sequence (Viterbi)
    estimatedStates = viterbi_path(prior1, transmat1, E);

    % Change point detection
    change_points = find(abs(diff(estimatedStates)) > 0);

    %disp(change_points);
    %disp(size(change_points,2));

    % remove Kevin Murphy's HMM Matlab toolbox
    % to prevent error when use 'legend' function
    rmpath(genpath('/home/leejang/lib/HMMall'));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plot
    figure;

    plot1 = subplot(2,1,1);
    plot(sub_data(:,1),sub_data(:,3),...
         sub_data(:,1),sub_data(:,4),...
         sub_data(:,1),sub_data(:,5),...
         sub_data(:,1),sub_data(:,6),...
         sub_data(:,1),sub_data(:,7),...
         sub_data(:,1),sub_data(:,8),...
         sub_data(:,1),sub_data(:,9));
    title 'Force Sensing Data';
    xlabel 'Time stamp';
    ylabel 'Force (N)';
    legend('Force 0','Force 1','Force 2','Force 3',...
           'Force 4','Force 5','Force 6','Location','NorthEast');

    ylim = get(gca, 'ylim');
    for i = 1:size(change_points,2)
        h(i) = line([change_points(i)+400 change_points(i)+400], ylim);
    end
    
    plot2 = subplot(2,1,2);
    imagesc(sub_data(:,1)',estimatedStates,estimatedStates);
    set(gca,'ytick',[]);
    p = get(plot2,'position');
    disp(p);
    p(2) = 0.3; % bottom of the subplot
    p(4) = p(4) * 0.3; % dcrease size of height as 30%
    disp(p);
    set(plot2,'position',p);

    saveas(gcf, 'segment_output.jpg');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(0)
    % do kmenas clustring
    idx = kmeans(force_data_wo_time, 6);

    figure;
    [silh,h] = silhouette(force_data_wo_time, idx);
    h = gca;
    h.Children.EdgeColor = [.8 .8 1];
    xlabel 'Silhouette Value'
    ylabel 'Cluster'
    saveas(gcf, 'culstering.jpg');

    %disp(size(force_data_wo_time));
end

if(0)
    % plot
    figure;
    plot(force_data(:,1),force_data(:,3),...
         force_data(:,1),force_data(:,4),...
         force_data(:,1),force_data(:,5),...
         force_data(:,1),force_data(:,6),...
         force_data(:,1),force_data(:,7),...
         force_data(:,1),force_data(:,8),...
         force_data(:,1),force_data(:,9));
    title 'Force Sensing Data';
    xlabel 'Time stamp';
    ylabel 'Force (N)';
    legend('Force 0','Force 1','Force 2','Force 3',...
           'Force 4','Force 5','Force 6','Location','NorthWest');
    saveas(gcf, 'force_output.jpg');
end
    disp('done');
end

function force_data = parse_force_data(force_data_file)

    fID = fopen(force_data_file);

    data = textscan(fID,'%f %f %f %f %f %f %f %f %f %f');
    force_data = [data{1} data{2} data{3} data{4} data{6} data{7} data{8} data{9} data{10}];

    fclose(fID);
end

