%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

%{
%%% 函数作用
% 已知每一个数据集中的每一个tag的四个角点，得到这个可能是边缘检测算法，角点检测算法或者人工选择得到的。(论文里有引用相应的文献)
% 对于每一个tag的四个角点，组成四条边，对于每一条边来说，将其扩充到一个矩形。
% 然后选择这个矩形内 的 二值图的边缘点，然后根据这些点，和ransac算法拟合出每一个线的方程。
% 然后每两个线的交点就是我们要找的 camera_tag 的角点
%}

function BagData = refineImageCorners(path, BagData, skip_indices, display, t_clean)
    if nargin > 4
        clean = t_clean;
    else
        clean = 1;
    end
    extend_fac = 2;%边缘扩充为矩形的一个参数
    corner_array = [1 1 2 3
                    2 3 4 4];
    
    for k = 1:size(BagData, 2)%对每一个bag文件
        if any(ismember(k, skip_indices))%k 里面有一个值属于skip_indices，就跳过
            continue
        end
        file = BagData(k).bagfile;
        %选择里面bag文件前1s的图像数据 并读取
        bagselect = rosbag(path + file);
        bagselect2 = select(bagselect,'Time',...
            [bagselect.StartTime bagselect.StartTime + 1],'Topic','/camera/color/image_raw');
        allMsgs = readMessages(bagselect2);
        [img,~] = readImage(allMsgs{1});
        %转为灰度图，用canny算子提取边缘
        gray = rgb2gray(img);
        BW = edge(gray, 'Canny', [0.04]);
        
        %创建两个图像句柄，一个显示原图，一个显示边缘图
        if checkDisplay(display)
            figure(1000)
            if clean
                clf('reset')
                imshow(img)
            end
            hold on

            title(file)
            
            figure(2000)
            if clean
                clf('reset')
                imshow(BW)
            end
            hold on
            title(file)
        end
        for j = 1:BagData(k).num_tag%第k个数据集中的有num_tag个tag，对每个tag进行操作
            if checkDisplay(display)
%                 disp("Before modification")
%                 disp([BagData(k).camera_target(j).corners])
            end
            for i = 1: length(corner_array)%值为4，对于每一个边进行操作 i=1时表示第一个点和第二个点之间的边
                %对于该边的两个定点
                p1 = BagData(k).camera_target(j).corners(1:2, corner_array(1,i));
                p2 = BagData(k).camera_target(j).corners(1:2, corner_array(2,i));
                
                vec = [p1 - p2];
                vec_normlized = vec/norm(vec);%该边的方向向量
                vec_p = [vec_normlized(2); -vec_normlized(1)];%垂直的方向向量
                %向外扩充边缘
                p1_ext = p1 + extend_fac * vec_normlized + extend_fac * vec_p;
                p2_ext = p2 - extend_fac * vec_normlized + extend_fac * vec_p;

                p3_ext = p2 - extend_fac * vec_normlized - extend_fac * vec_p;
                p4_ext = p1 + extend_fac * vec_normlized - extend_fac * vec_p;
                

                corners = [p1_ext, p2_ext, p3_ext, p4_ext];%边缘扩充之后的矩形框
                [x2, y2] = poly2cw(corners(1,:)', corners(2,:)');% 把四个按照 顺时针的顺序排布
                [img_y, img_x] = size(gray);
                x_dim = linspace(1, img_x, img_x);  %[1,2,...,img_x]
                x_dim = repmat(x_dim,1,img_y);      %[1,2,...,img_x,...,1,2,...,img_x]一共复制img_y次
                y_dim = linspace(1, img_y, img_y);
                y_dim = repelem(y_dim,1, img_x);    %[1,2,...,img_y,...,1,2,...,img_y]一共复制img_x次

                [in, ~] = inpolygon(x_dim, y_dim, x2, y2);% 返回的矩阵in是640*480 X 1的矩阵，里面的值为1的元素代表点在"corners表示的轮廓"内
                x_dim = x_dim(in);%在矩形轮廓内的点的x坐标
                y_dim = y_dim(in);%在矩形轮廓内的点的y坐标,在下面有显示
                
                %将矩形轮廓里面的点中 是边缘点的提取出来
                data = [];
                for t = 1:size(x_dim, 2)
                    if BW(y_dim(t), x_dim(t))%边缘点为白色
                        data = [data, [x_dim(t); y_dim(t)]];
                    end
                end
                [x, y, line_model, inlier_pts] = ransacLineWithInlier(data', 0.1);% line_model 是y = k * x + b中的k和b
                
                if checkDisplay(display)
                    figure(2000)
                    hold on
                    scatter(BagData(k).camera_target(j).corners(1,:), BagData(k).camera_target(j).corners(2,:))
                    scatter(p1(1), p1(2))
                    scatter(p2(1), p2(2))
                    scatter(p1_ext(1), p1_ext(2))
                    scatter(p2_ext(1), p2_ext(2))
                    scatter(p3_ext(1), p3_ext(2))
                    scatter(p4_ext(1), p4_ext(2))
                    scatter(x_dim, y_dim, 'g.');
                    scatter(inlier_pts(:,1), inlier_pts(:,2), 'm.');
%                     plot(x, y, '-', 'LineWidth',2, 'MarkerSize',10, 'color', [0.8500, 0.3250, 0.0980])
                    plot(x, y, '-', 'LineWidth',2, 'MarkerSize',10, 'color', [0, 1, 1])
                    
                    figure(1000)
                    hold on 
                    scatter(BagData(k).camera_target(j).corners(1,:), BagData(k).camera_target(j).corners(2,:))
                    scatter(p1(1), p1(2))
                    scatter(p2(1), p2(2))
                    plot(x, y, '-', 'LineWidth',2, 'MarkerSize',10, 'color', [0.8500, 0.3250, 0.0980])
                end
                
                t_edge(i).x = x;%x是该边两个端点的横坐标
                t_edge(i).y = y;%y是该边两个端点的纵坐标
                t_edge(i).line = line_model;
            end
            
            % 根据ransac得到的边缘的直线方程求交点，也就是最后代入优化问题的交点
            cross_big_2d = [];
            for i = 1: length(corner_array)
                point = intersection(t_edge(corner_array(1,i)).line, t_edge(corner_array(2,i)).line);
                cross_big_2d = [cross_big_2d, point];
            end
            cross_big_2d = sortrows(cross_big_2d', 2, 'ascend')';
            cross_big_2d = [cross_big_2d; ones(1, 4)];
            BagData(k).camera_target(j).corners = cross_big_2d;
            
            if checkDisplay(display)
%                 disp("Afrer modification")
%                 disp([BagData(k).camera_target(j).corners])
                figure(2000)
                scatter(cross_big_2d(1,:), cross_big_2d(2,:), 'filled');
                hold off

                figure(1000)
                scatter(cross_big_2d(1,:), cross_big_2d(2,:), 'g','filled');
                hold off
                drawnow
            end
        end
        if checkDisplay(display)
%             disp("press any key to continue")
%             pause;
%             clc
        end
    end
end
