function mat_Points = RequestPoints(TPS, distmode)
    tmp = 1;
    mat_Points = [];
    while(tmp ~= 0)
        prompt = 'Do you want to take a measurement? [Y/N] ';
        str = input(prompt,'s');
        if (isempty(str)|| str == 'Y') 
            [Hz, V, D] = get(TPS,distmode)
            mat_Points = [mat_Points, [Hz, V, D]'];
        elseif str == 'N'
            tmp = 0;
        end
        
    end

end
