function changeFace (TPSport)
  %Switch from front to back sight
  req = '%R1Q,9028:';
  fprintf (TPSport, req); 

  %Reply - read response data into the sting-variable out
  out = fscanf(TPSport);

  %Extract the returned values
  data = regexp(out,':(?<RC>[^,]*)','names');
       
  %Backside measurements set
  if(str2num(data.RC)==0)   
      fprintf ('Change face sucessfully done');
  else
      fprintf ('[ERROR] Change face failed');
  end
end