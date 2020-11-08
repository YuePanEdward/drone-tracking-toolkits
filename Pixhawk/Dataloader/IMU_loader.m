% Read IMU data
% time_boot_ms	time_usec	xacc	yacc	zacc	xgyro	ygyro	zgyro	xmag	ymag	zmag

close all; clc;
clear data
log = 'test_log2.log';

for i = 1:numel(log)
    log_str = char(log(i));
    %var_str = log_str(2:end)erase(log_str, '_label');
    lbl_group.LineNo = 0;
    if contains(log_str,'label') && ~exist(var_str, 'var')
        clear(log_str); 
    else
        if contains(log_str, 'label') && ~contains(log_str, ['ans','log'])
            var = erase(log_str, '_label');
            temp_var = eval(var);
            [~, col] = size(temp_var);
            temp_lbl = eval(log_str);
            for j = 1:col
                lbl = char(temp_lbl(j,1));                
                lbl_group.(lbl) = temp_var(:,j);
            end
        end
        data.(var_str) = lbl_group;
        lbl_group = rmfield(lbl_group, fieldnames(lbl_group));
    end
end

[n,~] = size(PARM);
for i = 1:n
    data.PARM.(char(PARM(i,1))) = PARM{i,2};
end

clearvars -except data
