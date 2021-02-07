%% Audio analysis : find the triggering pattern
% audio processing toolkit needed
% By Yue Pan @ ETHZ IGP

%%
video_base_path = 'G:\1_data\dataset_20210119\video';

% For test experiment
% video_name = 'sony_a5100.mp4';   % CAM 0
% video_name = 'huawei_p40.mp4';   % CAM 1
% video_name = 'gopro7.mp4';           % CAM 2
% video_name = 'sony_G.mts';           % CAM 3
% video_name = 'samsung.mp4';        % CAM 4
video_name = 'sony_nex5n.mts';         % CAM 5

video_path =  [video_base_path filesep video_name];

fps=29.97; % frame rate

%fileReader = dsp.AudioFileReader("Filename",video_path);

%%
[audioData,fs] = audioread(video_path);
%soundsc(audioData,fs);  % play the audio, fs is the sample rate


%%
duration = size(audioData,1) / fs;

fprintf("The audio duriation is %4.2f (s)\n", duration);

%% 
sample_time = ((0:size(audioData,1)-1)*1/fs)'; % begin:step:end
frame_duration = 1/fps; % unit: s
frame_count = ceil(duration/frame_duration); 
frame_time = 0:frame_duration:frame_count*frame_duration;    % begin:step:end

tsignal = audioData(:,2);

%% filter

% fsignal = fft(tsignal);
% L=size(tsignal,1);
% 
% P2 = abs(fsignal/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% 
% f =fs*(0:(L/2))/L;

%% High-pass or Band-pass filter
% reference: https://ch.mathworks.com/help/signal/ref/highpass.html
% reference: https://ch.mathworks.com/help/signal/ref/bandpass.html

threshold_down = 5000;
threshold_up = 5500;
hp_tsignal = highpass(tsignal,threshold_down,fs);

bp_tsignal = bandpass(tsignal,[threshold_down, threshold_up],fs);

%% plot
figure (1);
%plot (sample_time, bp_tsignal, 'Color',[1 ,0.5, 0]); % IPad, use audioData(:,1)
plot (sample_time, tsignal, 'Color',[1 ,0.5, 0]); % IPad, use audioData(:,1)
% stem (tp_trigger, ones(1,size(tp_trigger,1)), 'k','LineWidth',3);
%plot (sample_time,tsignal,'Color',[0.8500 ,0.3250, 0.0980]); % IPad, use audioData(:,1)
width=2000;
height=300;
set(gcf,'position',[10,100,width,height])
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
grid on;
%xticks(frame_time);
% ylim([-0.1, 0.1]);
xlim([75, 90]);
xlabel('time (s)','Fontname', 'Times New Roman','FontSize',14);
ylabel('Signal amplitude','Fontname', 'Times New Roman','FontSize',14);
%title(['Audio signal: ', strrep(video_name,'_','\_')],'Fontname', 'Times New Roman','FontSize',16);

% title('Triggering timestamp in project time system','Fontname', 'Times New Roman','FontSize',16);

%% ffmpeg: extract audio from video (Linux toolkit)
% ffmpeg -i input-video.mp4 output-audio.mp3
