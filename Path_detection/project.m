%Remove noise from the given image
%add path
addpath('D:\New folder (2)');

%Load image
img = imread('Above_tunnel.png');
% imshow(img);


im_test = zeros(428,835);

for row = 8:436
    for col = 1:835
        % control green ratio in image
        if(img(row,col,2)>47 && img(row,col,2)< 120 )
            im_test(row,col) = 1;
        end
    end
end

% removing small elements
BW2 = bwareaopen(im_test,100000);

% Edge detection in the processed image
th = 50;
edge_canny = edge(BW2, 'Canny');
edge_sobel = edge(BW2, 'Sobel');

edge_roberts = edge(BW2, 'Roberts');

edge_prewitt = edge(BW2, 'Prewitt');


% figure
%  subplot(2,2,1);imshow(edge_canny);
%  title('Canny');
% subplot(2,2,2);imshow(edge_sobel);
% title('Sobel');
% subplot(2,2,3);imshow(edge_roberts);
% title('Roberts');
% subplot(2,2,4);imshow(edge_prewitt);
% title('Prewitt');

%Get the rows and cols of the valid pixels from the edge image
[rows, cols] = size(edge_canny);
data = [];
for colEdge = 1:cols
    for rowEdge = 1: (rows)
        if edge_canny(rowEdge, colEdge) == 1
            data = [data [colEdge;rowEdge]];
             
        end 
    end
end

%Dividing data set into right and left
sz = size(data, 2);
sz1 = round(sz*0.8);

% use function ransac line to get most dominant line

num = 2;
iter = 5000;
threshDist = 2;
inlierRatio = 0.1;

figure;
imshow(img); hold on;
%plot(data(1,1:sz1),data(2,1:sz1),'o');hold on;
%plot(data(1,:),data(2,:),'o');hold on;
[BM1, BM2] = ransac_line(data(:, 1:sz1), num,iter,threshDist,[]);
[BM3, BM4] = ransac_line(data(:, sz1:end), num,iter,threshDist,[]);

