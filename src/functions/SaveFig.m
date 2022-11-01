%Save the figure

function SaveFig(fig, name_)
    % File name
    name = num2str(fig.Number); 
    if nargin == 2
        name = name_;
    end

    % Save figure as .png
    folderName = 'data/png';
    if ~exist(folderName, 'dir') % Create folder if not exist
        mkdir(folderName)
    end
    filePath = [folderName '/' name '.png']; % File path
    % saveas(fig, filePath) % With default resolution
    exportgraphics(fig, filePath, 'Resolution', 500) % With specific resolution

    % Save figure as .fig
    folderName = 'data/fig';
    if ~exist(folderName, 'dir') % Create folder if not exist
        mkdir(folderName)
    end
    filePath = [folderName '/' name '.fig']; % File path
    savefig(filePath)

end