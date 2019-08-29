function [Ergodicity_Metric] = Calculate_Ergodicity(ck, opt)


LK= opt.erg.LK;
muk = opt.erg.muk;
Ergodicity_Metric =  sum(sum( LK .* (ck - muk).^2 ));
end
% function [Ergodicity_Metric] = Calculate_Ergodicity(Ck, opt)
% 
% Nkx = opt.erg.Nkx;
% Nky = opt.erg.Nkx;
% muk = opt.erg.muk;
% 
% Ergodicity_Metric=0;
%     for kx = 0:Nkx-1
%         for ky = 0:Nky-1
%             lambda_k = 1.0 / ((1.0 + kx * kx + ky * ky)^1.5);
%             Ergodicity_Metric=Ergodicity_Metric+lambda_k*(abs(Ck(kx+1, ky+1)-muk(kx+1, ky+1)))^2;
%         end
%     end
% %     display('Ergodicity_Metric:')
% end



