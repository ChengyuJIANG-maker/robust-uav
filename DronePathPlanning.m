% clear;
 clc;
 close all;

%% 1. 参数配置
mapSize   = [50 50 20];      % 地图大小 [x y z]，单位：栅格
resolution= 1;               % 每个栅格代表的实际米数（可改）
start     = [2 2 2];         % 起点栅格坐标
goal      = [48 48 18];      % 终点栅格坐标
obsRatio  = 0.15;            % 初始随机障碍物占比（0~1）


useSmooth  = true;           % 是否进行三次样条平滑
useGUI     = true;           % 是否启用交互式障碍物编辑

%% 2. 初始化地图
map = zeros(mapSize);
fprintf('正在生成 %d%% 随机障碍物...\n', round(obsRatio*100));
N = round(prod(mapSize)*obsRatio);
while nnz(map) < N
    % 分别为 x/y/z 指定随机范围
    obs = [randi([1 mapSize(1)]), ...
           randi([1 mapSize(2)]), ...
           randi([1 mapSize(3)])];
    sz  = randi([2 4],1,3);
    map = addBoxObs(map, obs-sz, obs+sz);
end

%% 3. 交互式编辑障碍物
if useGUI
    % 新建普通 figure，避免 Dock 或 App 模式
    hFig = figure('Name','双击增删障碍物，回车继续', ...
                  'NumberTitle','off', ...
                  'DockControls','off', ...
                  'MenuBar','none', ...
                  'ToolBar','none');
    ax   = axes('Parent',hFig,'NextPlot','add');
    showMap3D(map,start,goal,ax);
    view(3); grid on; axis equal;
    
    % 关闭冲突的交互工具
    rotate3d(ax,'off'); zoom(ax,'off'); pan(ax,'off');
    
    % 监听鼠标双击
    set(hFig,'WindowButtonDownFcn',@mouseCallback);
    set(hFig,'KeyPressFcn',@keyCallback);
    
    fprintf('双击图中任意位置增删障碍物（3×3×3 块），按回车继续...\n');
    uiwait(hFig);           % 等待回车
    close(hFig);            % 安全关闭
end

%% 4. A* 搜索
tic;
[path, expand] = Astar3D(map, start, goal);
tCost = toc;
if isempty(path)
    error('未找到可行路径！请减少障碍物或调整起终点。');
end

%% 5. 路径后处理
origLen = pathLength(path);
if useSmooth
    path = smoothPath(path, map);
end
smoothLen = pathLength(path);

%% 6. 可视化结果
fprintf('A* 完成！扩展节点: %d, 耗时: %.3fs\n', size(expand,1), tCost);
fprintf('原始长度: %.2f m, 平滑后: %.2f m\n', origLen*resolution, smoothLen*resolution);
%% 6. 可视化结果
fprintf('A* 完成！扩展节点: %d, 耗时: %.3fs\n', size(expand,1), tCost);
fprintf('原始长度: %.2f m, 平滑后: %.2f m\n', origLen*resolution, smoothLen*resolution);

% 强制新建独立窗口，并置顶
fig3d = figure('Name','3D 航迹', ...
               'NumberTitle','off', ...
               'WindowState','normal', ...
               'Position',[360 240 800 600]);
ax3d  = axes('Parent',fig3d);
showMap3D(map,start,goal,ax3d); hold(ax3d,'on');
plot3(ax3d,expand(:,1),expand(:,2),expand(:,3),'y.','MarkerSize',2);
plot3(ax3d,path(:,1),path(:,2),path(:,3),'r','LineWidth',2);
plot3(ax3d,path(:,1),path(:,2),path(:,3),'go','MarkerSize',4,'MarkerFaceColor','g');
view(ax3d,3); grid(ax3d,'on'); axis equal; rotate3d on;
legend(ax3d,'障碍物','起点','终点','A* 扩展节点','航迹','航点');
showMap3D(map,start,goal,[]); hold on;
plot3(expand(:,1),expand(:,2),expand(:,3),'y.','MarkerSize',2); % CLOSE 集
plot3(path(:,1),path(:,2),path(:,3),'r','LineWidth',2);
plot3(path(:,1),path(:,2),path(:,3),'go','MarkerSize',4,'MarkerFaceColor','g');
view(3); grid on; axis equal; rotate3d on;
legend('障碍物','起点','终点','A* 扩展节点','航迹','航点');

%% 7. 2D 投影补充视图
figure('Name','俯视 / 侧视投影');
subplot(1,2,1); imagesc(squeeze(any(map,3))'); colormap(gray); axis equal ij;
hold on; plot(path(:,1),path(:,2),'r','LineWidth',2);
plot(start(1),start(2),'go','MarkerSize',8,'MarkerFace','g');
plot(goal(1), goal(2), 'rx','MarkerSize',10,'LineWidth',2);
title('俯视图 (XY)'); xlabel('X'); ylabel('Y');

subplot(1,2,2); imagesc(squeeze(any(map,2))'); colormap(gray); axis equal ij;
hold on; plot(path(:,1),path(:,3),'r','LineWidth',2);
plot(start(1),start(3),'go','MarkerSize',8,'MarkerFace','g');
plot(goal(1), goal(3), 'rx','MarkerSize',10,'LineWidth',2);
title('侧视图 (XZ)'); xlabel('X'); ylabel('Z');

%% -------------- 子函数 --------------

function map = addBoxObs(map,c1,c2)
% 将长方体区域标记为障碍
c1 = max(ceil(c1),1); c2 = min(floor(c2),size(map));
map(c1(1):c2(1),c1(2):c2(2),c1(3):c2(3)) = 1;
end

function showMap3D(map,start,goal,ax)
if nargin<4 || isempty(ax); figure; ax = axes; end
% 绘制障碍物
[x,y,z] = ind2sub(size(map),find(map==1));
scatter3(x,y,z,30,'filled','MarkerFaceColor',[0.3 0.3 0.3],'Parent',ax);
hold(ax,'on');
scatter3(start(1),start(2),start(3),80,'go','MarkerFaceColor','g','Parent',ax);
scatter3(goal(1),goal(2),goal(3),80,'rx','MarkerFaceColor','r','Parent',ax);
end

function [path,expand] = Astar3D(map,start,goal)
% 三维 A* 算法
start = double(start); goal = double(goal);
mapSize = size(map);
dirs = [0 0 0; ...
        1 0 0;-1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1; ...
        1 1 0;1 -1 0;-1 1 0;-1 -1 0; ...
        1 0 1;1 0 -1;-1 0 1;-1 0 -1; ...
        0 1 1;0 1 -1;0 -1 1;0 -1 -1; ...
        1 1 1;1 1 -1;1 -1 1;1 -1 -1;-1 1 1;-1 1 -1;-1 -1 1;-1 -1 -1];
cost = sqrt(sum(dirs.^2,2)); % 欧氏距离代价
dirs = dirs';
N = size(dirs,2);

% 初始化
openSet = start;                 % 开放列表
gScore  = inf(prod(mapSize),1);  % 线性索引
fScore  = gScore;
parent  = zeros(prod(mapSize),1);
idxS = sub2ind(mapSize,start(1),start(2),start(3));
idxG = sub2ind(mapSize,goal(1),goal(2),goal(3));
gScore(idxS) = 0;
fScore(idxS) = norm(start-goal);

expand = []; % 记录扩展节点用于可视化

while ~isempty(openSet)
    % 选取 fScore 最小节点
    [~,idx] = min(fScore(sub2ind(mapSize,openSet(:,1),openSet(:,2),openSet(:,3))));
    current = openSet(idx,:);
    idxC = sub2ind(mapSize,current(1),current(2),current(3));
    
    % 到达目标
    if isequal(current,goal)
        path = reconstructPath(parent,idxC,mapSize);
        path = path(end:-1:1,:);
        return;
    end
    
    openSet(idx,:) = [];
    fScore(idxC) = inf;
    expand(end+1,:) = current;
    
    % 遍历 26 邻域
    for k = 1:N
        neighbor = current + dirs(:,k)';
        if any(neighbor<1 | neighbor>mapSize) || map(neighbor(1),neighbor(2),neighbor(3))
            continue; % 越界或障碍
        end
        idxN = sub2ind(mapSize,neighbor(1),neighbor(2),neighbor(3));
        tentG = gScore(idxC) + cost(k);
        if tentG < gScore(idxN)
            parent(idxN) = idxC;
            gScore(idxN) = tentG;
            fScore(idxN) = tentG + norm(neighbor-goal);
            if ~any(ismember(openSet,neighbor,'rows'))
                openSet(end+1,:) = neighbor;
            end
        end
    end
end
path = []; % 无路径
end

function path = reconstructPath(parent,idx,mapSize)
% 从终点回溯到起点
path = [];
while idx ~= 0
    [x,y,z] = ind2sub(mapSize,idx);
    path(end+1,:) = [x y z];
    idx = parent(idx);
end
end

function d = pathLength(path)
% 计算路径长度
d = sum(sqrt(sum(diff(path).^2,2)));
end

function path = smoothPath(path,map)
% 简单共线剪枝 + 三次样条平滑
% 1. 共线剪枝
idx = 1;
for k = 2:size(path,1)-1
    v1 = path(k,:) - path(idx,:);
    v2 = path(k+1,:) - path(k,:);
    if abs(dot(v1,v2)/(norm(v1)*norm(v2)+eps) - 1) < 1e-3
        continue;
    else
        idx = idx+1;
        path(idx,:) = path(k,:);
    end
end
path(idx+1,:) = path(end,:);
path = path(1:idx+1,:);

% 2. 三次样条平滑
t = 1:size(path,1);
ts = linspace(1,size(path,1),5*size(path,1));
path = [spline(t,path(:,1),ts)' spline(t,path(:,2),ts)' spline(t,path(:,3),ts)'];
% 检查碰撞
for k = 2:size(path,1)
    p = round(path(k,:));
    if any(p<1) || any(p>size(map)) || map(p(1),p(2),p(3))
        % 有碰撞则回退到原始离散路径
        path = round(path);
        break;
    end
end
end

%% --------- 交互回调 ----------
function mouseCallback(src,~)
% 双击增删障碍物
if strcmp(get(src,'SelectionType'),'normal')   % 左键双击
    pt = get(gca,'CurrentPoint');              % [x y z]
    pt = round(pt(1,1:3));
    map = evalin('base','map');                % 从主工作区获取变量
    mapSize = size(map);
    if all(pt>=1 & pt<=mapSize)
        % 切换 3×3×3 区域
        c1 = max(pt-1,1);
        c2 = min(pt+1,mapSize);
        map(c1(1):c2(1),c1(2):c2(2),c1(3):c2(3)) = ...
            1 - map(c1(1),c1(2),c1(3));        % 0↔1
        assignin('base','map',map);            % 写回工作区
        % 刷新图形
        cla; showMap3D(map,evalin('base','start'),evalin('base','goal'),gca);
        view(3); grid on; axis equal;
    end
end
end

function keyCallback(src,evt)
if strcmp(evt.Key,'return')
    uiresume(src);
end
end