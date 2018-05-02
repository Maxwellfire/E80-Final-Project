function smoothedVector = RunningAvg(inputVector,numAvgs)
% Smooths out the input vector using a running avg of length numAvgs

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
    smoothedVector(i) = sum(inputVector(startIndex:endIndex))/(endIndex-startIndex+1);
end


end

