function points = get_elipse(P, scale)
    % function necessary just for the EKF plot given in the course
  
    N_points = 100;
    theta = linspace(0, 2*pi, N_points);
    circle = [cos(theta); sin(theta)];
    

    [U, S, ~] = svd(P);
    radii = scale * sqrt(diag(S));
    

    T = U * [radii(1) 0; 0 radii(2)];
    
 
    points = T * circle;
end