function [Ergodicity_Metric] = Calculate_Ergodicity(ck, muk, DomainBounds,Nagents)

Nkx = size(muk, 1);
Nky = size(muk, 2);

Ergodicity_Metric=0;
for iagent = 1:Nagents
    for kx = 0:Nkx-1
        for ky = 0:Nky-1
            lambda_k = 1.0 / ((1.0 + kx * kx + ky * ky)^1.5);     
            Ergodicity_Metric=Ergodicity_Metric+lambda_k*(abs(ck(kx+1, ky+1)-muk(kx+1, ky+1)))^2;
        end
    end
end

end


 