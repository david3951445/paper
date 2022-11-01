function Layout = GetTiledlayout(DIM)
    % Obtain a suitable size   
    %-1 Method 1: Average. ex. 12 -> 4by3
    % div = divisors(DIM);
    % i = ceil((length(div))/2);
    % Layout = tiledlayout(DIM/div(i), div(i));

    %-2 Method 2: 2 col. ex. 12 -> 6by2
    Layout = tiledlayout(DIM/2, 2); % (row: DIM/2, col: 2)

    % Fill the figure window
    Layout.TileSpacing = 'tight';
    Layout.Padding = 'tight';
end