%% Audio analysis

%% ffmpeg: extract audio from video
% ffmpeg -i input-video.mp4 output-audio.mp3

%%

video_base_path = '/home/edward/Videos/IPA';
video_base_path = 'C:\Users\20153\Videos';
%indoor test
video_name = 'Huawei_test_1.mp4';
video_name= 'Sony_test_1.mp4';
video_name = 'Ipad_test_1.MOV';
%outdoor test
video_name = 'Huawei_test_2.mp4';
video_name = 'Sony_test_2.mp4';
video_name = 'Ipad_test_2.MOV';

video_path =  [video_base_path filesep video_name];

fps=29.97;

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

%% plot
plot (sample_time,audioData(:,2),'Color',[0    0.4470    0.7410]); % IPad, use audioData(:,1)
%plot (sample_time,audioData(:,2),'Color',[0.8500 ,0.3250, 0.0980]); % IPad, use audioData(:,1)
width=1500;
height=250;
set(gcf,'position',[0,0,width,height])
set(gca, 'Fontname', 'Times New Roman','FontSize',12);
grid on;
%xticks(frame_time);
ylim([-0.1, 0.1]);
xlim([0, duration]);
xlabel('time (s)','Fontname', 'Times New Roman','FontSize',14);
ylabel('Signal amplitude','Fontname', 'Times New Roman','FontSize',14);
title(['Audio signal: ', strrep(video_name,'_','\_')],'Fontname', 'Times New Roman','FontSize',16);

