%Save the figure

function SaveFig(fig)
    % Save figure as .png
    FILE_NAME = ['data/png/fig' num2str(fig.Number) '.png']; % File name
    % saveas(fig, FILE_NAME) % With default resolution
    exportgraphics(fig, FILE_NAME, 'Resolution', 500) % With specific resolution

    % Save figure as .fig
    FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig']; % File name
    savefig(FILE_NAME)
end