clear; clc; close all;
format longG
%% Parameters
fs = 20e6;
ts = 1/fs;
videoSource = 'traffic.mp4';
vid = VideoReader(videoSource);
frame = readFrame(vid);
vid.CurrentTime = 0;
frame_mat = zeros(1,vid.NumFrames*vid.Height*vid.Width);
result_mat = zeros(1,vid.NumFrames*3*100);
%% Main loop
for frameCount = 1 : vid.NumFrames
    fprintf('Frame = %d\n', frameCount);
    frame = rgb2gray(readFrame(vid));
    frame_mat(1+(frameCount-1)*vid.Height*vid.Width:frameCount*vid.Height*vid.Width) = reshape(frame',1,vid.Height,vid.Width);
    hista = process_frame(frame, frameCount);
    result_mat(1+(frameCount-1)*300:frameCount*300)=reshape(single(hista(1:3,:)),1,[]);
    disp_point(frame, frameCount,hista);
end
% save_file1(frame_mat,'frame_matrix.bin');
save_file(result_mat,'result_matrix.bin');
%% ------------------------------------------------------------------------
function hista = process_frame(frame,frameCount)
persistent age
persistent state
persistent history
persistent ws
persistent p
persistent prevframe
persistent active
MAX_TRACKS = 100;
if frameCount == 1
    [active,age,state,history,ws,p] = init_track(MAX_TRACKS);
    num1 = 0;num2 = 0;num3 = 0;num4 = 0;num5 = 0;
else
    points = detector(frame,prevframe);num1=size(points,1);
    [active,age,state,history,ws,p] = add_track(active,age,state,history,ws,p, points);num2 = sum(active);
    for i = 1:MAX_TRACKS
        if active(i)
            [state(:,i),p(:,i)]= track_predict(state(:,i),p(:,i));
        end
    end
    [trackedPoints, isFound] = correlation(prevframe, frame,active,state);
    [age,active,state,ws] = edit_track(active,age,trackedPoints,isFound,ws,state);num3 = sum(active);
    active = merge_tracks(active,state,age);num4 = sum(active);
    active = cut_tracks(active,history,state,age);num5 = sum(active);
end
prevframe = frame;
hista = [active;state(1:2,:);history;ws];
[num1 num2 num3 num4 num5]
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [active,age,state,history,ws,p]  = init_track(MAX_TRACKS)
active = zeros(1,MAX_TRACKS);
age = zeros(1,MAX_TRACKS);
state = zeros(7,MAX_TRACKS);
history = zeros(2,MAX_TRACKS);
ws = zeros(1,MAX_TRACKS);
p = zeros(49,MAX_TRACKS);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [active,age,state,history,ws,p] = add_track(active,age,state,history,ws,p,points)
for i = 1:size(points,1)
    idx = find(active == 0, 1);
    if isempty(idx)
        break;
    end
    %
    active(idx) = true;
    age(idx) = 1;
    state(1,idx) = points(i,1);
    state(2,idx) = points(i,2);
    state(3,idx) = 0;
    state(4,idx) = 0;
    state(5,idx) = 0;
    state(6,idx) = 0;
    state(7,idx) = 1;
    history(1,idx) = points(i,1);
    history(2,idx) = points(i,2);
    ws(idx) = 20;
    p(1,idx)=1e-2;
    p(9,idx)=1e-2;
    p(17,idx)=1e-2;
    p(25,idx)=1e-2;
    p(32,idx)=1e-2;
    p(41,idx)=1e-2;
    p(49,idx)=1e-2;
end
end
%% ------------------------------------------------------------------------
function [state,p] = track_predict(state,p)
n = 7;
w = 1.0 / (2*n);
A = [1 0 0.033     0 0.0005445           0     0;
    0 1     0 0.033         0   0.0005445     0;
    0 0     1     0     0.033           0     0;
    0 0     0     1         0       0.033     0;
    0 0     0     0         1           0     0;
    0 0     0     0         0       0.033     0;
    0 0     0     0         0           0 0.033];
p = reshape(p, n, n) + 1e-6 * eye(n);
L = my_chol(p);
X = zeros(n, 2*n);
for i = 1:n
    X(:,i)     = state + L(:,i);
    X(:,i+n)   = state - L(:,i);
end
X_pred = zeros(n, 2*n);
for i = 1:2*n
    X_pred(:,i) = A*X(:,i);
end
x_pred2 = sum(X_pred, 2) * w;

Q = diag([1e-2, 1e-2, 1e-1, 1e-1, 1e-3, 1e-3, 1e-4]);
diff = X_pred - x_pred2;
P_pred = Q + w * (diff * diff');
state = round(x_pred2);
p = P_pred(:);
end
%% ------------------------------------------------------------------------
function L = my_chol(P)
n = size(P, 1);
L = zeros(n, n);
for i = 1:n
    for j = 1:i
        sum_k = 0;
        for k = 1:j-1
            sum_k = sum_k + L(i,k) * L(j,k);
        end
        if i == j
            L(i,i) = sqrt(P(i,i) - sum_k);
        else
            L(i,j) = (P(i,j) - sum_k) / L(j,j);
        end
    end
end
end
%% ------------------------------------------------------------------------
function [age,active,state,ws] = edit_track(active,age,z,isFound,ws,state)
MAX_TRACKS = 100;
for i = 1:MAX_TRACKS
    if active(i)
        if isFound(i) == 1
            age(i) = age(i) + 1;
            state(1:2,i) = z(i,:);
            state(7,i) = max(0.5, min(5,state(7,i)));
        else
            active(i) = 0;
        end
    end
end
end
%% ------------------------------------------------------------------------
function active = merge_tracks(active,state,age)
MAX_TRACKS = 100;
for i = 1:MAX_TRACKS
    if ~active(i), continue; end
    for j = i+1:MAX_TRACKS
        if ~active(j), continue; end
        diff_traks = sqrt(sum((state(1:2,i)-state(1:2,j)).^2));
        if diff_traks < 40
            if age(i) >= age(j)
                active(j) = false;
            else
                active(i) = false;
            end
        end
    end
end
end
%% ------------------------------------------------------------------------
function active = cut_tracks(active,history,state,age)
MAX_TRACKS = 100;
for i = 1:MAX_TRACKS
    if active(i)
        diff_pos = mean(abs(history(:,i) - state(1:2,i)));
        if diff_pos < age(i)
            active(i) = false;
        else
            if age(i) > 100
                active(i) = false;
            end
        end
    end
end
end
%%
function disp_point(frame, frameCount,hista)
MAX_TRACKS = 100;
figure(1);
imshow(frame, []);
hold on;
for i = 1:MAX_TRACKS
    if hista(1,i)
        plot(hista(4,i), hista(5,i), 'yo','MarkerSize',20, 'LineWidth', 1);
        plot(hista(2,i), hista(3,i), 'go', 'MarkerSize',20, 'LineWidth',1);
        line([hista(4,i) hista(2,i)],[hista(5,i) hista(3,i)],'Color','red','LineWidth',2);
    end
end
title(sprintf('Frame %d', frameCount));
hold off;
drawnow;
end
%% helper functions
function save_file(x,name)
fid = fopen(name, 'wb');
fwrite(fid,x, 'float32');
fclose(fid);
end
%%
function save_file1(x,name)
fid = fopen(name, 'wb');
fwrite(fid,uint8(x), 'uint8');
fclose(fid);
end

