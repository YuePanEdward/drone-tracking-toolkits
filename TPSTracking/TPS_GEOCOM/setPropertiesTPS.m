function setPropertiesTPS(TPSport, prism_type, target_type, atr_state, hz_tol, v_tol)
% Change prism type
    if ~isempty(prism_type)
        req=['%R1Q,17008:',num2str(prism_type)];
        fprintf (TPSport, req);
        out = fscanf (TPSport);
        data = regexp(out,':(?<RC>[^,]*)','names');

% Check if the prism type has been changed
        if str2num(data.RC) == 0
            fprintf('Prism type changed\n');
        else
            fprintf('Error in changing prism type\n');  
        end 
    end
% Change target type (EDM mode)
    if ~isempty(target_type)
        req=['%R1Q,17021:',num2str(target_type)];
        fprintf (TPSport, req);
        out = fscanf (TPSport);
        data = regexp(out,':(?<RC>[^,]*)','names');

% Check if the target type has been changed
        if str2num(data.RC) == 0
            fprintf('EDM mode changed\n');
        else
            fprintf('Error in changing EDM mode type\n');
        end
    end

% Change ATR state
    if ~isempty(atr_state)
        req=['%R1Q,18005:',num2str(atr_state)];
        fprintf (TPSport, req);
        out = fscanf (TPSport);
        data = regexp(out,':(?<RC>[^,]*)','names');

% Check if the automatic mode is on
        if str2num(data.RC) == 0
            fprintf('ATR state changed\n');
        else
            fprintf('Error in changing ATR state type\n');
        end
    end

% Set tolerances
    if ~(isempty(hz_tol)||isempty(v_tol)) 
        req=['%R1Q,9007:',num2str(hz_tol), ',', num2str(v_tol)];
        fprintf (TPSport, req);
        out = fscanf (TPSport);
        data = regexp(out,':(?<RC>[^,]*)','names');

%Check if both tolerance has been set up
        if str2num(data.RC) == 0
            fprintf('Horizontal and vertical tolerances set\n');
        else
            fprintf('Error in changing tolerances\n');
        end

    end
end 