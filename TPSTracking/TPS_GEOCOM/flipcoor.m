function mat_rev = flipcoor(Hz,V)
    
%     Hz(Hz+pi >2*pi) = Hz(Hz+pi >2*pi)-pi; 
%     Hz(Hz+pi <= 2*pi) = Hz(Hz+pi <= 2*pi) + pi; 
Hz_rev = []
for ind = 1:size(Hz,2)
    if(Hz(ind)+pi >2*pi)
        Hz_rev(1,ind) = Hz(ind)-pi;
        
    else
         Hz_rev(1,ind) = Hz(ind)+pi;
    end
end    
    
    V_rev = 2*pi - V;
    
    mat_rev = [Hz_rev;V_rev];

end