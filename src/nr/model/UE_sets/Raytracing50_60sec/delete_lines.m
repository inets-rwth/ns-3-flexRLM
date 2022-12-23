function delete_lines()

dinfo = dir('*.txt');
formatSpec_cords = '%.2f,%.2f,1.5\n';

for i = 1:100
    
    filename = dinfo(i).name;
    data = load(filename);
    
    newData = data(startLine:length(data),:);
    fid = fopen(filename, 'w');
    
    if fid == -1
        error('Cannot open file for writing: %s', filename);
    end
    
    dim = size(newData);
    if dim(2) > 1
        for j=1:length(newData)
            fprintf(fid, formatSpec_cords, newData(j,1), newData(j,2));
        end
    else
        for k=1:length(newData)
            fprintf(fid, '%.2f\n', newData(k));
        end
    end
    
    fclose(fid);
end

fclose('all');
end