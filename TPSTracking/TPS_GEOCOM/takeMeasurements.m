function mat_meas  = takeMeasurements(TPSport, distmode, mat_points)
%%First side
meas_front = []; 
% Request - send a command to the total station to set Hz=0, V=100gon
    for ind=1:size(mat_points,2)
        fprintf('\nAUT_MakePositioning - turning the telescope to a specified position (Hz=%sgon, V=%sgon)\n',...
            num2str(mat_points(1,ind)),num2str(mat_points(2,ind)));
%Send request to move the TLS with the angles contain in matrix Points 
        req=['%R1Q,' sprintf('9027:%s,%s,0,0',num2str(mat_points(1,ind)),num2str(mat_points(2 ,ind)))];
        fprintf (TPSport, req); 

% Reply - read response data into the sting-variable out
            out = fscanf (TPSport); 

% Extract the returned values
        data = regexp(out,':(?<RC>[^,]*)','names');

        if str2num(data.RC) == 0
            fprintf('Telescope turned to Hz=%.4f and V=%.4f\n\n',mat_points(1,ind),mat_points(2,ind) );
        end
%Take measurement        
        [Hz, V, D] = getAngles(TPSport, distmode);
        meas_front = [meas_front,[Hz;V;D]]
    
    end 
    
%% Back side
%Switch from front to back sight
  req = '%R1Q,9028:';
        fprintf (TPSport, req); 

% Reply - read response data into the sting-variable out
        out = fscanf(TPSport);

% Extract the returned values
       data = regexp(out,':(?<RC>[^,]*)','names');
       
%Backside measurements set

    if(str2num(data.RC)  == 0)   
        fprintf ('Change to back face sucessfully done');
%Calculate the coordinate of the reverse matrix for angles
        meas_back = []
        mat_rev = flipcoor(mat_points(1,:),mat_points(2,:));
        
        for ind=size(mat_points,2):-1:1
            fprintf('\n Phase 2 starts - turning the telescope to a specified position (Hz=%sgon, V=%sgon)\n',...
                num2str(mat_rev(1,ind)),num2str(mat_rev(2,ind))); 

%Send request to move the TLS with the angles contain in matrix Points 
            req=['%R1Q,' sprintf('9027:%s,%s,0,0',num2str(mat_rev(1,ind)),num2str(mat_rev(2 ,ind)))];
            fprintf (TPSport, req); 

% Reply - read response data into the sting-variable out
            out = fscanf (TPSport); 

    % Extract the returned values
            data = regexp(out,':(?<RC>[^,]*)','names');
            
            if str2num(data.RC) == 0
                fprintf('Telescope turned to Hz=%.4f and V=%.4f\n\n',mat_rev(1,ind),mat_rev(2,ind) );
            end
%Take measurement        
            [Hz, V, D] = getAngles(TPSport, distmode);
            meas_back = [meas_back,[Hz;V;D]];
    
        end
    end
    mat_meas(:,:,1) = meas_front;
    mat_meas(:,:,2) = meas_back;
    fprintf("Done.\n");
end