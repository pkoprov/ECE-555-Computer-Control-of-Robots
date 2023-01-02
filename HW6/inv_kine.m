function [q,n, p_sse] = inv_kine(Rob, goal)

    qk = zeros(1,7);
    doLoop = true;
    n=1;
    while doLoop
        FK = Rob.fkine(qk);
        pose_k = [FK.t', FK.tr2rpy];
        pos_error = pose_k - goal;
    
        p_sse = sqrt(sum(pos_error(1:3).^2));        

        if all(abs(pos_error(4:5)) <= 0.05) && p_sse <= 0.01 
            doLoop = false;
        end
    
        J = Rob.jacob0(qk);
        J_inv = pinv(J);
        Xv = (goal - pose_k)';
        Q = J_inv*Xv;
        
        qk = qk+Q';
        n=n+1;
        if mod(n,1000) == 0
            disp("Iteration #" +num2str(n))
            disp("Possitional error: " + num2str((pos_error)))
            disp("Velocity vector: " + num2str(Xv'))
        end
    end
    q = wrapToPi(qk);