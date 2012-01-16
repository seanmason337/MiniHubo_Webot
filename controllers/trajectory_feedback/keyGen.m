function key = keyGen(size, start, delt)
key = zeros(size,1);
for i = 1:size
    key(i) = start +i*delt;
end