function [trackedPoints, isFound] = correlation(prevImg, currImg,active,state)
MAX_TRACKS = 100;
prevImg = double(prevImg);
currImg = double(currImg);
isFound = false(MAX_TRACKS, 1);
trackedPoints = zeros(MAX_TRACKS, 2);
win = 10;
wins = 7;
n = (2*win+1)*(2*win+1);
%
for t = 1:MAX_TRACKS
    if active(t)
        pp = round(state(1:2,t));
        corr_xy = zeros(2*win+1, 2*win+1);
        if pp(1)-win >= 1 && pp(2)-win >= 1 && pp(2)+win <= size(currImg,1) && pp(1)+win <= size(currImg,2)
            x = prevImg(pp(2)-win:pp(2)+win, pp(1)-win:pp(1)+win);
            sx = sum(x(:));
            sxx = sum(x(:).*x(:));
            for i = -wins:wins
                flag = 1;
                for j = -wins:wins
                    if flag ==1 && pp(2)-win+i >= 1 && pp(2)+win+i <= size(currImg,1) && pp(1)-win+j >= 1 && pp(1)+win+j <= size(currImg,2)
                        y = currImg(pp(2)-win+i:pp(2)+win+i, pp(1)-win+j:pp(1)+win+j);
                        sy = sum(y(:));
                        syy = sum(y(:).*y(:));
                        sxy = sum(x(:).*y(:));
                        k1 = n*sxy - sx*sy;
                        k2 = n*sxx - sx*sx;
                        k3 = n*syy - sy*sy;
                        co = abs(k1) / sqrt(k2*k3 + eps);
                        corr_xy(i+win+1, j+win+1) = co;
                        if co < 0.5
                            flag = 0;
                        end
                    else
                        corr_xy(i+win+1, j+win+1) = 0;
                    end
                end
            end
            corr_xy(isnan(corr_xy)) = 0;
            ma = (max(corr_xy(:)));
            [ii, jj] = find((corr_xy) == ma, 1);
            ii = ii - win - 1;
            jj = jj - win - 1;
            jj = min(max(pp(1) + jj, 1), size(currImg,2));
            ii = min(max(pp(2) + ii, 1), size(currImg,1));
            ma = round(1000*ma);
            if ma > 900
                isFound(t) = true;
                trackedPoints(t,1) = jj;
                trackedPoints(t,2) = ii; 
            else
                isFound(t) = false;
            end
        end
    end
end
end