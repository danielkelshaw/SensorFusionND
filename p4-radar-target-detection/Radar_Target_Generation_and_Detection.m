clear all;
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1m
% Max Velocity = 100m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

radar_max_range = 200;
radar_max_velocity = 100;
radar_range_resolution = 1;
c_light = 3e8;

%% User Defined Range and Velocity of target

% Define the target's initial position and velocity.
target_range = 40;
target_velocity = 60;

%% FMCW Waveform Generation

% Bandwidth, Chirp Time, Slope of the FMCW
B = c_light / (2 * radar_range_resolution);
Tchirp = 5.5 * 2 * radar_max_range / c_light;
slope = B / Tchirp;

% Operating carrier frequency of Radar
fc = 77e9;
                                                        
% Number of chirps in one sequence
Nd = 128;

% Number of samples on each chirp
Nr = 1024;

% Time array for running scenarios for every sample on each chirp
t = linspace(0, Nd * Tchirp, Nr * Nd);

% Vectors for Tx, Rx and Mix 
Tx = zeros(1, length(t));
Rx = zeros(1, length(t));
Mix = zeros(1, length(t));

% Vectors for range covered and time delay
r_t = zeros(1, length(t));
td = zeros(1, length(t));

%% Signal generation and Moving Target simulation

% Running the radar scenario over the time
for i = 1:length(t)
    
    % Update the range covered for constant velocity. 
    r_t(i) = target_range + target_velocity * t(i);
    td(i) = 2 * r_t(i) / c_light;
    
    % Update Tx / Rx
    Tx(i) = cos(2 * pi * (fc * t(i) + 0.5 * slope * t(i) ^ 2));
    Rx(i) = cos(2 * pi * (fc * (t(i) - td(i)) + 0.5 * slope * (t(i) - td(i)) ^ 2));
    
    % Mix Tx / Rx signals
    Mix(i) = Tx(i) .* Rx(i);
    
end

%% RANGE MEASUREMENT

% Reshape the vector into Nr * Nd array
Mix = reshape(Mix, [Nr, Nd]);

% Run the FFT on the beat signal along Nr and normalize
signal_fft2 = fft(Mix, Nr);
signal_fft2 = signal_fft2 ./ Nr;

% Take the absolute value of FFT output
signal_fft2 = abs(signal_fft2);

% Convert to single-sided signal
ss_signal_fft = signal_fft2(1:Nr/2);

% Plotting the range
figure('Name','Range from First FFT')
plot(ss_signal_fft);
axis([0 200 0 1]);

%% RANGE DOPPLER RESPONSE

Mix = reshape(Mix, [Nr, Nd]);

% 2D FFT using the FFT size for both dimensions
signal_fft2 = fft2(Mix, Nr, Nd);

% Taking just one side of signal from Range dimension
signal_fft2 = signal_fft2(1:Nr/2, 1:Nd);
signal_fft2 = fftshift(signal_fft2);

RDM = abs(signal_fft2);
RDM = 10 * log10(RDM);

% Plot the output of 2DFFT
doppler_axis = linspace(-100, 100, Nd);
range_axis = linspace(-200, 200, Nr/2) * ((Nr/2) / 400);
figure, surf(doppler_axis, range_axis, RDM);

%% CFAR implementation
% Slide Window through the complete Range Doppler Map

% Select the number of Training Cells in both the dimensions
Tr = 12;
Td = 5;

% Select the number of Guard Cells in both dimensions around the 
% Cell under Test (CUT) for accurate estimation
Gr = 4;
Gd = 3;

% Offset the threshold by SNR value in dB
offset = 1.4;

% Converting bin sizes to range and doppler based on max values
signal_post_thresholding = RDM / max(max(RDM));

% Loop to slide CUT across range doppler map
for i = (Tr + Gr + 1):((Nr / 2) - (Gr + Tr))
    for j = (Td + Gd + 1):(Nd - (Gd + Td))
        
        noise_level = zeros(1, 1);
        
        % Sum the signal level within all the Training Cells
        for p = (i - (Tr + Gr)):(i + (Tr + Gr))
            for q = (j - (Td + Gd)):(j + (Td + Gd))
                if (abs(i - p) > Gr || abs(j - q) > Gd)
                    noise_level = noise_level + db2pow(signal_post_thresholding(p, q));
                end
            end
        end
        
        % Average the noise level and convert to dB then add an offset
        threshold = pow2db(noise_level / (2 * (Td + Gd + 1) * 2 * (Tr + Gr + 1) - (Gr * Gd) - 1));
        threshold = threshold + offset;
        
        % If CUT is less than threshold set to zero, otherwise one
        CUT = signal_post_thresholding(i, j);
        if (CUT < threshold)
            signal_post_thresholding(i, j) = 0;
        else
            signal_post_thresholding(i, j) = 1;
        end
        
    end
end

% Set edge cells to zero as sliding window reduces size
signal_post_thresholding(union(1:(Tr + Gr), (end - (Tr + Gr - 1)):end), :) = 0; 
signal_post_thresholding(:, union(1:(Td + Gd), (end - (Td + Gd - 1)):end)) = 0;

% Display the CFAR output
figure, surf(doppler_axis, range_axis, signal_post_thresholding);
colorbar;
