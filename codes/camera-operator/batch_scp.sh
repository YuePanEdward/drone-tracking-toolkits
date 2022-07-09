video_folder_name=video00001

cam0_ip=10
cam1_ip=11
cam2_ip=12
cam3_ip=13
cam4_ip=14
cam5_ip=15

scp pi@192.168.0.${cam0_ip}:/home/pi/videos/${video_folder_name}/video.avi ./videos/${video_folder_name}/cam_${cam0_ip}.avi
scp pi@192.168.0.${cam1_ip}:/home/pi/videos/${video_folder_name}/video.avi ./videos/${video_folder_name}/cam_${cam1_ip}.avi
scp pi@192.168.0.${cam2_ip}:/home/pi/videos/${video_folder_name}/video.avi ./videos/${video_folder_name}/cam_${cam2_ip}.avi
scp pi@192.168.0.${cam3_ip}:/home/pi/videos/${video_folder_name}/video.avi ./videos/${video_folder_name}/cam_${cam3_ip}.avi
#scp pi@192.168.0.${cam4_ip}:/home/pi/videos/video00001/video.avi .
#scp pi@192.168.0.${cam5_ip}:/home/pi/videos/video00001/video.avi .