function smoothedVector = MedianGuy(inputVector,numAvgs)
% Smooths out the input vector by returning just the median of a running
% window
vectorLength = length(inputVector);
smoothedVector = inputVector; % output is the same size as the input

if numAvgs < 1
    numAvgs = 1;
elseif mod(numAvgs,2) == 0
    numAvgs = numAvgs - 1;
end

halfWindow = floor(numAvgs/2);

for i=1:vectorLength
    startIndex = i-halfWindow;
    endIndex = i+halfWindow;
    if startIndex < 1
        startIndex = 1;
    end
    if endIndex > vectorLength
        endIndex = vectorLength;
    end
    smoothedVector(i) = median(inputVector(startIndex:endIndex));
end


end


