function [mean] = square_mean(Vector)
    mean = sqrt(sum(Vector.^2))/length(Vector(:,1));

end