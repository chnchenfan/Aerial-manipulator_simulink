function [sys,x0,str,ts] = sfunc_tracking_differentiator(t,x,u,flag)
%SFUNC_TRACKING_DIFFERENTIATOR 跟踪微分器离散S-Function
%
% 功能：实现韩京清教授提出的跟踪微分器算法
%      可有效抑制噪声，提供平滑的微分信号
%
% 作者：技术支持团队
% 版本：1.0
% 日期：2024年
%
% 输入参数 u:
%   u(1) - 输入信号（需要微分的信号）
%
% 输出参数 y:
%   y(1) - 跟踪信号（滤波后的输入）
%   y(2) - 微分信号（输入信号的导数）
%
% 状态变量 x:
%   x(1) - 跟踪状态 x1
%   x(2) - 微分状态 x2
%
% 参数说明：
%   r - 速度因子（越大跟踪越快，建议范围：10-1000）
%   h - 滤波因子（越大滤波效果越好，建议范围：0.001-0.1）

switch flag
    case 0 % 初始化
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 2 % 离散状态更新
        sys = mdlUpdate(t,x,u);
    case 3 % 输出计算
        sys = mdlOutputs(t,x,u);
    case {1,4,9} % 其他情况
        sys = [];
    otherwise
        error('跟踪微分器S-Function: 未处理的flag值: %d', flag);
end

%=============================================================================
% mdlInitializeSizes: 系统初始化
%=============================================================================
function [sys,x0,str,ts] = mdlInitializeSizes

% 获取TD参数（可根据需要调整）
params = get_td_parameters();

sizes = simsizes;
sizes.NumContStates  = 0;   % 无连续状态
sizes.NumDiscStates  = 2;   % 2个离散状态 [x1; x2]
sizes.NumOutputs     = 2;   % 2个输出 [跟踪信号; 微分信号]
sizes.NumInputs      = 1;   % 1个输入
sizes.DirFeedthrough = 1;   % 无直接馈通（避免代数环）
sizes.NumSampleTimes = 1;   % 一个采样时间

sys = simsizes(sizes);

% 初始离散状态

x0 = [0; 0];  % [x1(0); x2(0)]

str = [];                    % 无状态名称
ts = [params.Ts 0];         % 离散时间系统，采样时间Ts

% 创建调试日志文件
create_debug_log();

%=============================================================================
% mdlUpdate: 离散状态更新
%=============================================================================
function sys = mdlUpdate(t,x,u)

% 获取参数
params = get_td_parameters();

% 解析输入
v = u(1);  % 输入信号
% 解析状态
x1 = x(1);  % 跟踪状态
x2 = x(2);  % 微分状态

% 计算误差
e = x1 - v;

% === 核心算法：最速控制综合函数 ===
fh = fhan(e, x2, params.r, params.h);

% 状态更新方程（离散化）
x1_next = x1 + params.Ts * x2;
x2_next = x2 + params.Ts * fh;

% 更新状态
sys = [x1_next; x2_next];

% 记录调试信息（每100个采样周期记录一次）
persistent counter;
if isempty(counter)
    counter = 0;
end
counter = counter + 1;

if mod(counter, 100) == 0
    log_debug_info(t, v, x1, x2, fh);
end

%=============================================================================
% mdlOutputs: 输出方程
%=============================================================================
function sys = mdlOutputs(t,x,u)
v = u(1);
% v = NaN;
% if(v > -1 && v<1 )
%     fprintf("1");
% else
%     v = 0;
% end
% if(isnan(v) )
%     v = 0;
% end

% 输出
sys = [x(1);x(2)];  % y2: 微分信号（输入的导数）
%=============================================================================
% 辅助函数
%=============================================================================

function params = get_td_parameters()
% 获取跟踪微分器参数
%
% 这些参数可根据实际系统调整：
% - 快速响应系统：r=500-1000, h=0.001-0.005
% - 平滑系统：r=50-200, h=0.01-0.05
% - 强噪声环境：r=10-50, h=0.05-0.1

params.r = 100;    % 速度因子
params.h = 0.2;   % 滤波因子
params.Ts = 0.01;  % 采样时间（10ms）

function fh = fhan(x1, x2, r, h)
% 最速控制综合函数
% 这是跟踪微分器的核心算法
%
% 输入：
%   x1 - 位置误差
%   x2 - 速度状态
%   r  - 速度因子
%   h  - 滤波因子
%
% 输出：
%   fh - 控制量

% 计算关键参数
d = r * h;
d0 = h * d;
y = x1 + h * x2;
a0 = sqrt(d^2 + 8 * r * abs(y));

% 计算控制量
if abs(y) <= d0
    % 线性区（原点附近）
    fh = -r * y / h;
else
    % 非线性区（快速收敛）
    if y > 0
        a = (a0 - d) / 2;
    else
        a = (a0 + d) / 2;
    end
    
    fh = -r * sign(y) * a / d;
    
    % 速度限制
    if abs(x2) > d
        fh = -r * sign(x2);
    end
end

function create_debug_log()
% 创建调试日志文件

filename = 'td_debug_log.txt';
fid = fopen(filename, 'w');
if fid ~= -1
    fprintf(fid, '=== 跟踪微分器调试日志 ===\n');
    fprintf(fid, '创建时间: %s\n\n', datestr(now));
    fprintf(fid, '%-12s %-12s %-12s %-12s %-12s\n', ...
            '时间(s)', '输入信号', '跟踪信号', '微分信号', '控制量');
    fprintf(fid, '%s\n', repmat('-', 70, 1));
    fclose(fid);
end

function log_debug_info(t, input, x1, x2, fh)
% 记录调试信息到文件

filename = 'td_debug_log.txt';
fid = fopen(filename, 'a');
if fid ~= -1
    fprintf(fid, '%-12.4f %-12.6f %-12.6f %-12.6f %-12.6f\n', ...
            t, input, x1, x2, fh);
    fclose(fid);
end

%=============================================================================
% 扩展功能：自适应参数调整（可选）
%=============================================================================

function params = adaptive_parameters(e, params)
% 根据误差大小自适应调整参数
% 大误差时加快响应，小误差时增强滤波

e_abs = abs(e);
if e_abs > 1.0
    % 大误差：快速响应
    params.r = min(500, params.r * 1.2);
    params.h = max(0.001, params.h * 0.8);
elseif e_abs < 0.1
    % 小误差：平滑滤波
    params.r = max(50, params.r * 0.9);
    params.h = min(0.1, params.h * 1.1);
end