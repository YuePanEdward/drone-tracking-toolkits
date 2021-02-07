% video_ts: get the framewise timestamp of a video
% video processing toolkit needed

video_base_path = 'G:\1_data\dataset_20210119\video';

% For test experiment
% video_name = 'sony_a5100.mp4';   % CAM 0
% video_name = 'huawei_p40.mp4';   % CAM 1
% video_name = 'gopro7.mp4';           % CAM 2
% video_name = 'sony_G.mts';           % CAM 3
% video_name = 'samsung.mp4';        % CAM 4
video_name = 'sony_nex5n.mts';         % CAM 5


video_path =  [video_base_path filesep video_name];


%% to obtain timestamp for each frame in lice video for timeseries
 obj = VideoReader(video_path);   %video
 
 current_time = [];
 framecount = 0;
 while hasFrame(obj)
      readFrame(obj);
      framecount = framecount + 1;
      current_time(framecount) = obj.CurrentTime;
 end

 %%
 sonyn_frame_ts = sync_sonyn(1) * current_time' + sync_sonyn(2);
 frame_id = 1: framecount;
 sonyn_frame_ts = [frame_id'  sonyn_frame_ts];
