%Remove noise from the given image
%add path
addpath('D:\Vaibhav\Autonomous_UAV-master\Path_detection');

%add video
vid = VideoReader('Drone.mp4');
numFrames = vid.NumberOfFrames;
n=numFrames;
%frames to read  = 300

path = 'D:\Vaibhav\Autonomous_UAV-master\Path_detection\Images\';
for i = 1:2:300
    frames = read(vid,i);
    
    imwrite(frames, [path, 'Image' int2str(i),  '.jpg'])
    %imwrite(frames,['Image' int2str(i), '.jpg']);
    %im(i)=image(frames);
end
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

%% Get the rows and cols of the valid pixels from the edge image
[rows, cols] = size(edge_canny);
data = [];
for colEdge = 1:cols
    for rowEdge = 1: (rows)
        if edge_canny(rowEdge, colEdge) == 1
            data = [data [colEdge;rowEdge]];
             
        end 
    end
end
%% Dividing data set into right and left
sz = size(data, 2);
sz1 = round(sz*0.8);
%% show markers and image
figure;
%position of corner points of line in initial image
pos   = [354 5;42 328;838 328;419 5];
color = {'blue'};
RGB = insertMarker(img,pos,'+','color',color,'size',10);

%position of corner points of target image
pos   = [200 5;200 328;600 328;600 5];
color = {'white'};
RGB1 = insertMarker(RGB,pos,'+','color',color,'size',10);

imshow(img); hold on;
%imshow(RGB1); hold on;

%% use function ransac line to get most dominant line

num = 2;
iter = 5000;
threshDist = 2;
inlierRatio = 0.1;

%plot(data(1,1:sz1),data(2,1:sz1),'o');hold on;
%plot(data(1,:),data(2,:),'o');hold on;
[BM1, BM2] = ransac_line(data(:, 1:sz1), num,iter,threshDist,[]);
[BM3, BM4] = ransac_line(data(:, sz1:end), num,iter,threshDist,[]);

%% homography transform with input points and output points

im_input = [354 42 838 419; 5 328 328 5];
im_output = [100 100 400 400;5 328 328 5];
v = homography_solve(im_input, im_output);

%% call homography solve function with input points and homography matrix
figure;
res = zeros(442,841,3);
img = im2double(img);
for row = 1:442
    for col = 1:841
       ab = [col;row];
       y = round(homography_transform(ab, v));
       if (y(2)>0 && y(2)<=841) && (y(1)>0 && y(1)<=442)
       %warp_im(row,col) = im(get_pix(2), get_pix(1));
            res(y(2), y(1),:) = img(row, col,:);
       end
    end
end
imshow(res);

%get the 3rd dimension 
% check the output is correct or not