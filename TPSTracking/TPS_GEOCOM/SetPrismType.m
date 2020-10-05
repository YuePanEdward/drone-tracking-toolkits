function RC = SetPrismType(prismtype, TPSport)
    req=['%R1Q,17008:',num2str(prismtype)];
    fprintf (TPSport, req);
    out = fscanf (TPSport);
    data = regexp(out,':(?<RC>[^,]*)','names');
    RC = data.RC;
end