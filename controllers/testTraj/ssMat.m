function R = ssMat(size, n)
    R = zeros(size);
    
    for i = 1:size
        for j = 1:size
            
            if abs(j-i)>=n+1
                R(i,j) = -inf;
            end
        end
    end
    
end

