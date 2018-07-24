Img = imread('/home/soowon/RDKDC_Labs/ur5_Final_Project/bj.tif');
imshow(Img)
%%
figure;
imcontour(Img,3)
%%
clc
d1 = double(rgb2gray(imread('/home/soowon/RDKDC_Labs/ur5_Final_Project/bj.tif')))./255;  % Load the image, scale from 0 to 1
subplot(2, 2, 1); imshow(d1); title('d1');  % Plot the original image
d = edge(d1, 'canny', .6);                  % Perform Canny edge detection
subplot(2, 2, 2); imshow(d); title('d');    % Plot the edges
ds = bwareaopen(d, 40);                     % Remove small edge objects
subplot(2, 2, 3); imshow(ds); title('ds');  % Plot the remaining edges
iout = d1;
BW = ds;
iout(:, :, 1) = iout;                           % Initialize red color plane
iout(:, :, 2) = iout(:, :, 1);                  % Initialize green color plane
iout(:, :, 3) = iout(:, :, 1);                  % Initialize blue color plane
iout(:, :, 2) = min(iout(:, :, 2) + BW, 1.0);   % Add edges to green color plane
iout(:, :, 3) = min(iout(:, :, 3) + BW, 1.0);   % Add edges to blue color plane
subplot(2, 2, 4); imshow(iout); title('iout');  % Plot the resulting image

%%
AA = imcontour(d1);