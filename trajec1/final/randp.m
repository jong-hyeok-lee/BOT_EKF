function p=randp(scale)
lin = linspace(-scale, scale, 10000*scale);
idx = randi(length(lin));
p = lin(idx);
end