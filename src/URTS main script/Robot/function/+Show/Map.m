function Map()
    % Show the occupy map of URTS
    img = imread('data/map.png');     % Load a sample image
    xImage = [6.5 6.5;-.5 -.5]; % The x data for the image corners
    yImage = [-3 6; -3 6 ];   
    zImage = [ 0 0; 0 0 ];
    surf(xImage,yImage,zImage, 'CData', img, FaceColor='texturemap', EdgeColor='none');
end