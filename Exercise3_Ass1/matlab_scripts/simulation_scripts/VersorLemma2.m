%% VersorLemma function
% inputs:
% - r1: the first rotation matrix
% - r2: the second rotation matrix
% output:
% - c: the Vect3 representing the axis around which r1 should rotate to reach r2, where its modulus is the angle

function [c, theta] = VersorLemma2(r1, r2)

    % threshold used to approsimate theta
    threshold = 1 * 10^(-4);
     
    ia = r1(:, 1);
    ib = r2(:, 1);
    ja = r1(:, 2);
    jb = r2(:, 2);
    ka = r1(:, 3);
    kb = r2(:, 3);
    
    % here is computed the vector lemma using the geometric form
    % the dot function  returns the scalar product between the arguments
    % the cross function  returns the vector product between the arguments
    % the norm function returns the eucledian norm of the arguments
    vsinth = ( cross(ia, ib) + cross(ja, jb) + cross(ka, kb) ) / 2 ;
    sinth = norm( vsinth );
    costh = ( dot(ia, ib) + dot(ja, jb) + dot(ka, kb) - 1 ) / 2;
    
    % The cases used here are the same described in class
    % Case theta ~= 0
    if(costh >= (1 - threshold)) 
       v = [0, 0, 0]';
       theta = 0;
    % Case 0 < theta < pi
    elseif(abs(costh) < 1 - threshold) 
       theta = atan2(sinth, costh);
       v = vsinth / sinth;
    % Case theta ~ pi  
    else 
       theta = pi;
       v = [0, 0, 0]';
       R = r1(:,:) + r2(:,:);
       if(norm(R(:, 1) ) ~= 0)
           v = R(:, 1);
       end
       if(norm(R(:, 2)) > norm(v) )
           v = R(:, 2);
       end
       if(norm( R(:, 3) ) > norm(v))
          v = R(:, 3); 
       end    
    end
    
    c = v * theta;
end
