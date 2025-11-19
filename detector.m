function corners = detector(x,xx)
n = 5;
x = single(x);
[h, w] = size(x);
p = zeros(size(x),'single');
for i = n+1:5:h-n
    for j = n+1:w-n
        valid = abs(double(x(i,j)) - double(xx(i,j))) >= 30;

        if valid == 1
            y=x(i-n:i+n,j-n:j+n);
            sum1 = sum(sum(y));
            sum2 = single(sum(sum(y.*y)));
            p(i,j) = sum2-sum1*sum1/single(121);
        end
    end
end
p=round(sqrt(p));
cnt =0;
corners=[];
for i = n+1:5:h-n
    for j = n+1:w-n
        valid = round(abs(double(x(i,j)) - double(xx(i,j))))>=30;
        if valid == 1 
            b = sum(p(i,j) >  p(i,j-n:j+n));
            if b == 10 
                cnt =cnt + 1;
                corners(cnt,:)=[j i];
            end
        end
    end
end
cnt = min(cnt,50);
corners = corners(1:cnt,:);
end
