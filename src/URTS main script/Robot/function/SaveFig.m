function SaveFig(fig)
    % Save figure as .fig and .png
    FILE_NAME = ['results/fig' num2str(fig.Number) '.png']; % path
    saveas(fig, FILE_NAME)
    FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig']; % path
    savefig(FILE_NAME)
end