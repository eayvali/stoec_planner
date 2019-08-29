%% beam sensor image processing: Creat lookup table
table = cell(150,150,91);

img = im2double(rgb2gray(imread('beam.png')));
img = imresize(img,[opt.ng(1),opt.ng(2)]);
figure(5);
img = flip(img);
% opt.image = img;
bias_row = +6;
bias_col = +3;
imgTrans = imtranslate(img,[bias_row,bias_col]);
tic
for itheta=0:90
    imgRot = imrotate(imgTrans,-itheta*4);

    %%%%%%%%%%%%%%%%% warper  %%%%%%%%%%%%%%%%%
    [p3, p4] = size(imgRot);
    q1 = opt.ng(1);
    q2 = opt.ng(2);
    i3_start = floor((p3-q1)/2)+1; % or round instead of floor; using neither gives warning
    i3_stop = i3_start + q1-1;
    i4_start = floor((p4-q2)/2)+1;
    i4_stop = i4_start + q2-1;
    II = imgRot(i3_start:i3_stop, i4_start:i4_stop);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    II = imtranslate(II,[-75+75,-75 + 75]);
    II = flip(II);
    table{75,75,itheta+1} = sparse(II);
    imshow(II);
end
toc
% [x, y] = getpts
%% x and y 
tableX = cell(150);%%each cell contains 150x91 cells for a range of y and theta (for effcient parallel computing)
parpool('local',20)
tic
toc
parfor x = 1:150 
      tableY_THETA = cell(150,91);
      for y = 1:150
       for itheta=0:90
%            tic     
           II_original = full(table{75,75,itheta+1});
           tableY_THETA{y,itheta+1} = sparse(imtranslate(II_original,[-75 + x, +75 - y]));
%            toc
       end
       y
      end
   tableX{x} = tableY_THETA;
   x
end

%% Check results
for y = 100:150
    for itheta=0:90
        temp = tableX{100};
        II = full(temp{y,itheta+1});
        imshow(II);
    end
end

%% restore the results efficiently
for x = 1:150 
      for y = 1:150
       for itheta=0:90
       temp = tableX{x};
       II = temp{y,itheta+1};
       table{x,y,itheta+1} = II;
       end
      end
x      
end

%% Save lookup table
% tableSave = table(:,:,1);
tableX20 = tableX(1:20);
tableX40 = tableX(21:40);
tableX60 = tableX(41:60);
tableX80 = tableX(61:80);
tableX100 = tableX(81:100);
tableX120 = tableX(101:120);
tableX140 = tableX(121:140);
tableX150 = tableX(141:150);

save('lookup_table140.mat','tableX140','-v7.3');
save('lookup_table150.mat','tableX150','-v7.3');

display('Done!')

