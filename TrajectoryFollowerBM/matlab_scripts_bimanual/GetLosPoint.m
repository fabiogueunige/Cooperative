function Los = GetLosPoint(P, A, B)
    % input:
    % P: Position of object [x0, y0, z0]
    % A: Last target [x1, y1, z1]
    % B: Future target [x2, y2, z2]
    
    % output:
    % Los: Projection of robot on line moved of one unit vector on line
    P = P(1:3, 4);
    A = A(1:3, 4);
    B = B(1:3, 4);

    % Compute directional vector of line
    v = B - A;
    
    % Check if A and B are the same point (v is zero)
    if norm(v) < eps
        error('A and B are the same point. Cannot define a line.');
    end

    % Normalize the direction vector
    v_normalized = v / norm(v);

    % Compute parameter t
    t = dot(P - A, v) / dot(v,v);
    
    % Q: Projected point of robot on line [x, y, z]
    Q = A + t * v;
    
    % Los: Point moved of one unit vector on line
    Los = [eye(3), Q + (v_normalized * 0.02); 0, 0, 0, 1];
end