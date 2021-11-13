function matrix=anti_symmetric(x)
matrix=[0 -x(3) x(2);
        x(3) 0 -x(1);
        -x(2) x(1) 0];
    %eye(3)+sin(norm(x))/norm(x)*anti_symmetric(x)+sin(norm(x)/2)*sin(norm(x)/2)/(norm(x)*norm(x)/2)*anti_symmetric(x)*anti_symmetric(x);
end