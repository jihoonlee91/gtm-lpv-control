filetype = 'png';
resolution = 300;

% directory = [pwd,'\figure','\',filetype,num2str(resolution),'dpi'];      % Full path of the directory to be searched in
% filesAndFolders = dir(directory);     % Returns all the files and folders in the directory
% filesInDir = filesAndFolders(~([filesAndFolders.isdir]));  % Returns only the files in the directory            
% 
% for i = 1:numel(filesInDir)
%     figureName = filesInDir(i).name;
%     netName = filesInDir(i).name(1:end-4);    
%     delete [directory,'\',netName,'.',filetype];
% end

directory = [pwd,'\figure'];      % Full path of the directory to be searched in
filesAndFolders = dir(directory);     % Returns all the files and folders in the directory
filesInDir = filesAndFolders(~([filesAndFolders.isdir]));  % Returns only the files in the directory            

for i = 1:numel(filesInDir)
    figureName = filesInDir(i).name;
    open(figureName);
    netName = filesInDir(i).name(1:end-4);    
    exportgraphics(gcf,[directory,'\',filetype,num2str(resolution),'dpi\',netName,'.',filetype],'Resolution',resolution)
    close
end