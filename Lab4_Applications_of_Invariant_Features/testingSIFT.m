

dataPath = 'Results/';
folders = dir([dataPath 'Sequence*']);
letters = ['a', 'b', 'c', 'd'];
corr_p = [];
for i=1:length(folders)
    folder_name = folders(i).name;
    homFile = dir([dataPath folder_name '/*.mat']);
    original = [dataPath folder_name '/Image_00.pgm'];
    for k = 1:length(letters)
        images = dir([dataPath folder_name '/*', letters(k), '.pgm']);
        
        for j=1:length(images)
            img = [dataPath folder_name '/' images(j).name];
            load(homFile.name);
            
            img_num = str2num(images(j).name(7:8));
            
            if isempty(img_num) == 0
                if i==1
                    Hmatrix = Sequence1Homographies(img_num).H;
                elseif i==2
                    Hmatrix = Sequence2Homographies(img_num).H;
                else
                    Hmatrix = Sequence3Homographies(img_num).H;
                end
                
                [num, corr_p(k,j,i)] = correctMatches(original, img, Hmatrix);
                %clear Hmatrix;
            end
        end
        
    end
end



% plotting the correct matches 
for l=1:4
    figure(1)
    hold on;
    plot(1:16, corr_p(l,1:16,1));
    plot(1:16, four.corr_p(l,1:16,1));
    hold off;
    figure(2)
    hold on;
    plot(110:5:150, corr_p(l,1:9,2));
    plot(110:5:150, four.corr_p(l,1:9,2));
    hold off;
    figure(3)
    hold on;
    scatter(-45:5:45, corr_p(1,1:19,3));
    hold off
end












