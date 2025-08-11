// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//
// 导航函数计算
// 使用Dijkstra方法
// 修改为欧几里得距离计算
//
// 路径计算在没有插值的情况下进行，当附近单元格的势场达到最大值时
//
// 路径计算有一个检查其是否成功的逻辑
//

#include "nav2_navfn_planner/navfn.hpp"

#include <algorithm>
#include "rclcpp/rclcpp.hpp"

namespace nav2_navfn_planner
{
//
// 创建导航函数缓冲区
//

NavFn::NavFn(int xs, int ys)
{
  // 创建单元格数组
  costarr = NULL;
  potarr = NULL;
  pending = NULL;
  gradx = grady = NULL;
  setNavArr(xs, ys);

  // 优先级缓冲区
  pb1 = new int[PRIORITYBUFSIZE];
  pb2 = new int[PRIORITYBUFSIZE];
  pb3 = new int[PRIORITYBUFSIZE];

  // 对于Dijkstra（广度优先），设置为COST_NEUTRAL
  // 对于A*（最佳优先），设置为COST_NEUTRAL
  priInc = 2 * COST_NEUTRAL;

  // 目标和起点
  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;

  // 显示函数
  // displayFn = NULL;
  // displayInt = 0;

  // 路径缓冲区
  npathbuf = npath = 0;
  pathx = pathy = NULL;
  pathStep = 0.5;
}


NavFn::~NavFn()
{
  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }
  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }
  if (pathx) {
    delete[] pathx;
  }
  if (pathy) {
    delete[] pathy;
  }
  if (pb1) {
    delete[] pb1;
  }
  if (pb2) {
    delete[] pb2;
  }
  if (pb3) {
    delete[] pb3;
  }
}


//
// 设置导航函数的目标和起点位置
//

void
NavFn::setGoal(int * g)
{
  goal[0] = g[0];
  goal[1] = g[1];
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void
NavFn::setStart(int * g)
{
  start[0] = g[0];
  start[1] = g[1];
  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"), "[NavFn] Setting start to %d,%d\n", start[0],
    start[1]);
}

//
// 设置/重置地图大小
//

void
NavFn::setNavArr(int xs, int ys)
{
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Array is %d x %d\n", xs, ys);

  nx = xs;
  ny = ys;
  ns = nx * ny;

  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }

  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }

  costarr = new COSTTYPE[ns];  // 成本数组，2D配置空间
  memset(costarr, 0, ns * sizeof(COSTTYPE));
  potarr = new float[ns];  // 导航势能数组
  pending = new bool[ns];
  memset(pending, 0, ns * sizeof(bool));
  gradx = new float[ns];
  grady = new float[ns];
}


//
// 设置成本数组，通常来自ROS
//

void
NavFn::setCostmap(const COSTTYPE * cmap, bool isROS, bool allow_unknown)
{
  COSTTYPE * cm = costarr;
  if (isROS) {  // ROS类型的成本数组
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        // 这将转换传入的成本值:
        // COST_OBS                 -> COST_OBS (传入的“致命障碍”)
        // COST_OBS_ROS             -> COST_OBS (传入的“内切膨胀障碍”)
        // 0到252范围内的值 -> 从COST_NEUTRAL到COST_OBS_ROS的值。
        *cm = COST_OBS;
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS && allow_unknown) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  } else {  // 不是ROS地图，只是PGM
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        *cm = COST_OBS;
        if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
          continue;  // 不要处理边界
        }
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  }
}

//
// 给定成本图、目标和起点，计算导航函数
//

bool
NavFn::calcNavFnAstar()
{
  setupNavFn(true);

  // 计算导航函数和路径
  return propNavFnAstar(std::max(nx * ny / 20, nx + ny));
}

//
// 返回值
//

float * NavFn::getPathX() {return pathx;}
float * NavFn::getPathY() {return pathy;}
int NavFn::getPathLen() {return npath;}

// 插入到优先级块中
#define push_cur(n)  {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && curPe < PRIORITYBUFSIZE) \
    {curP[curPe++] = n; pending[n] = true;}}
#define push_next(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && nextPe < PRIORITYBUFSIZE) \
    {nextP[nextPe++] = n; pending[n] = true;}}
#define push_over(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && overPe < PRIORITYBUFSIZE) \
    {overP[overPe++] = n; pending[n] = true;}}


// 为新的传播设置导航势能数组

void
NavFn::setupNavFn(bool keepit)
{
  // 重置传播数组中的值
  for (int i = 0; i < ns; i++) {
    potarr[i] = POT_HIGH;
    if (!keepit) {
      costarr[i] = COST_NEUTRAL;
    }
    gradx[i] = grady[i] = 0.0;
  }

  // 成本数组的外部边界
  COSTTYPE * pc;
  pc = costarr;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }

  // 优先级缓冲区
  curT = COST_OBS;
  curP = pb1;
  curPe = 0;
  nextP = pb2;
  nextPe = 0;
  overP = pb3;
  overPe = 0;
  memset(pending, 0, ns * sizeof(bool));

  // 设置目标
  int k = goal[0] + goal[1] * nx;
  initCost(k, 0);

  // 查找障碍物单元格的数量
  pc = costarr;
  int ntot = 0;
  for (int i = 0; i < ns; i++, pc++) {
    if (*pc >= COST_OBS) {
      ntot++;  // 障碍物单元格的数量
    }
  }
  nobs = ntot;
}


// 初始化一个目标类型的成本以开始传播

void NavFn::initCost(int k, float v)
{
  potarr[k] = v;
  push_cur(k + 1);  // 右边的节点
  push_cur(k - 1);  // 左边的节点
  push_cur(k - nx); // 上边的节点
  push_cur(k + nx); // 下边的节点
  push_cur(k - nx - 1); // 左上角的节点
  push_cur(k - nx + 1); // 右上角的节点
  push_cur(k + nx - 1); // 左下角的节点
  push_cur(k + nx + 1); // 右下角的节点
}

// 关键函数：计算给定单元格的更新潜在值，
// 给定其邻居的值
// 从4个网格中的两个最低邻居进行平面波更新计算
// 对插值值进行二次近似
// 这里没有进行边界检查，这个函数应该很快
//

#define INVSQRT2 0.707106781

inline void NavFn::updateCell(int n)
{
  // 获取邻居节点的潜在值
  float l = potarr[n - 1];  // 左边的节点
  float r = potarr[n + 1];  // 右边的节点
  float u = potarr[n - nx]; // 上边的节点
  float d = potarr[n + nx]; // 下边的节点
  float ul = potarr[n - nx - 1]; // 左上角的节点
  float ur = potarr[n - nx + 1]; // 右上角的节点
  float dl = potarr[n + nx - 1]; // 左下角的节点
  float dr = potarr[n + nx + 1]; // 右下角的节点
  // ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
  //  potarr[n], l, r, u, d);
  // ROS_INFO("[Update] cost: %d\n", costarr[n]);

  // 找到最低的邻居节点及其最低的邻居
  float ta, tc;
  if (l < r) {tc = l;} else {tc = r;}  // 左右邻居中较小的值
  if (u < d) {ta = u;} else {ta = d;}  // 上下邻居中较小的值

  // 进行平面波更新
  if (costarr[n] < COST_OBS) {  // 不要传播到障碍物中
    float hf = static_cast<float>(costarr[n]);  // 可穿越性因子
    float dc = tc - ta;  // ta 和 tc 之间的相对成本
    if (dc < 0) {  // ta 是最低的
      dc = -dc;
      ta = tc;
    }

    // 计算新的潜在值
    float pot;
    if (dc >= hf) {  // 如果太大，使用 ta-only 更新
      pot = ta + hf;
    } else {  // 两个邻居的插值更新
      // 使用二次近似
      // 可能会通过表查找加快速度，但仍然需要进行除法
      float d = dc / hf;
      float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
      pot = ta + hf * v;
    }

    //      ROS_INFO("[Update] new pot: %d\n", costarr[n]);

    // 现在将受影响的邻居添加到优先级块中
    if (pot < potarr[n]) {
      float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
      float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
      float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
      float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);
      float ule = INVSQRT2 * static_cast<float>(costarr[n - nx - 1]);
      float ure = INVSQRT2 * static_cast<float>(costarr[n - nx + 1]);
      float dle = INVSQRT2 * static_cast<float>(costarr[n + nx - 1]);
      float dre = INVSQRT2 * static_cast<float>(costarr[n + nx + 1]);
      potarr[n] = pot;
      if (pot < curT) {  // 低成本缓冲块
        if (l > pot + le) {push_next(n - 1);}
        if (r > pot + re) {push_next(n + 1);}
        if (u > pot + ue) {push_next(n - nx);}
        if (d > pot + de) {push_next(n + nx);}
        if (ul > pot + ule) {push_next(n - nx - 1);}
        if (ur > pot + ure) {push_next(n - nx + 1);}
        if (dl > pot + dle) {push_next(n + nx - 1);}
        if (dr > pot + dre) {push_next(n + nx + 1);}
      } else {  // 溢出块
        if (l > pot + le) {push_over(n - 1);}
        if (r > pot + re) {push_over(n + 1);}
        if (u > pot + ue) {push_over(n - nx);}
        if (d > pot + de) {push_over(n + nx);}
        if (ul > pot + ule) {push_over(n - nx - 1);}
        if (ur > pot + ure) {push_over(n - nx + 1);}
        if (dl > pot + dle) {push_over(n + nx - 1);}
        if (dr > pot + dre) {push_over(n + nx + 1);}
      }
    }
  }
}


// 使用A*方法设置优先级
// 关键函数：计算给定单元格的更新潜在值，
// 给定其邻居的值
// 从4个网格中的两个最低邻居进行平面波更新计算
// 对插值值进行二次近似
// 这里没有进行边界检查，这个函数应该很快
//

#define INVSQRT2 0.707106781

inline void NavFn::updateCellAstar(int n)
{
  // 获取邻居节点的潜在值
  float l = potarr[n - 1];  // 左边的节点
  float r = potarr[n + 1];  // 右边的节点
  float u = potarr[n - nx]; // 上边的节点
  float d = potarr[n + nx]; // 下边的节点
  float ul = potarr[n - nx - 1]; // 左上角的节点
  float ur = potarr[n - nx + 1]; // 右上角的节点
  float dl = potarr[n + nx - 1]; // 左下角的节点
  float dr = potarr[n + nx + 1]; // 右下角的节点
  // ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
  // potarr[n], l, r, u, d);
  // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

  // 找到最低的邻居节点及其最低的邻居
  float ta, tc;
  if (l < r) {tc = l;} else {tc = r;}  // 左右邻居中较小的值
  if (u < d) {ta = u;} else {ta = d;}  // 上下邻居中较小的值

  // 进行平面波更新
  if (costarr[n] < COST_OBS) {  // 不要传播到障碍物中
    float hf = static_cast<float>(costarr[n]);  // 可穿越性因子
    float dc = tc - ta;  // ta 和 tc 之间的相对成本
    if (dc < 0) {  // ta 是最低的
      dc = -dc;
      ta = tc;
    }

    // 计算新的潜在值
    float pot;
    if (dc >= hf) {  // 如果太大，使用 ta-only 更新
      pot = ta + hf;
    } else {  // 两个邻居的插值更新
      // 使用二次近似
      // 可能会通过表查找加快速度，但仍然需要进行除法
      float d = dc / hf;
      float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
      pot = ta + hf * v;
    }

    // ROS_INFO("[Update] new pot: %d\n", costarr[n]);

    // 现在将受影响的邻居添加到优先级块中
    if (pot < potarr[n]) {
      float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
      float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
      float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
      float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);
      float ule = INVSQRT2 * static_cast<float>(costarr[n - nx - 1]);
      float ure = INVSQRT2 * static_cast<float>(costarr[n - nx + 1]);
      float dle = INVSQRT2 * static_cast<float>(costarr[n + nx - 1]);
      float dre = INVSQRT2 * static_cast<float>(costarr[n + nx + 1]);

      // 计算距离
      int x = n % nx;
      int y = n / nx;
      float dist = hypot(x - start[0], y - start[1]) * static_cast<float>(COST_NEUTRAL);

      potarr[n] = pot;
      pot += dist;
      if (pot < curT) {  // 低成本缓冲块
        if (l > pot + le) {push_next(n - 1);}
        if (r > pot + re) {push_next(n + 1);}
        if (u > pot + ue) {push_next(n - nx);}
        if (d > pot + de) {push_next(n + nx);}
        if (ul > pot + ule) {push_next(n - nx - 1);}
        if (ur > pot + ure) {push_next(n - nx + 1);}
        if (dl > pot + dle) {push_next(n + nx - 1);}
        if (dr > pot + dre) {push_next(n + nx + 1);}
      } else {
        if (l > pot + le) {push_over(n - 1);}
        if (r > pot + re) {push_over(n + 1);}
        if (u > pot + ue) {push_over(n - nx);}
        if (d > pot + de) {push_over(n + nx);}
        if (ul > pot + ule) {push_over(n - nx - 1);}
        if (ur > pot + ure) {push_over(n - nx + 1);}
        if (dl > pot + dle) {push_over(n + nx - 1);}
        if (dr > pot + dre) {push_over(n + nx + 1);}
      }
    }
  }
}

//
// 主传播函数
// A* 方法，最佳优先
// 使用欧几里得距离启发式
// 运行指定数量的周期，
//   或者直到没有更多的单元格可以更新，
//   或者直到找到起始单元格（atStart = true）
//

bool
NavFn::propNavFnAstar(int cycles)
{
  int nwv = 0;  // 最大优先级块大小
  int nc = 0;  // 放入优先级块的单元格数量
  int cycle = 0;  // 当前所在的周期

  // 设置初始阈值，基于距离
  float dist = hypot(goal[0] - start[0], goal[1] - start[1]) * static_cast<float>(COST_NEUTRAL);
  curT = dist + curT;

  // 设置起始单元格
  int startCell = start[1] * nx + start[0];

  // 主循环
  for (; cycle < cycles; cycle++) {  // 进行这么多周期，除非被中断
    if (curPe == 0 && nextPe == 0) {  // 优先级块为空
      break;
    }

    // 统计信息
    nc += curPe;
    if (curPe > nwv) {
      nwv = curPe;
    }

    // 重置当前优先级缓冲区的待处理标志
    int * pb = curP;
    int i = curPe;
    while (i-- > 0) {
      pending[*(pb++)] = false;
    }

    // 处理当前优先级缓冲区
    pb = curP;
    i = curPe;
    while (i-- > 0) {
      updateCellAstar(*pb++);
    }

    // 如果 (displayInt > 0 && (cycle % displayInt) == 0) {
    //   displayFn(this);
    // }

    // 交换优先级块 curP <=> nextP
    curPe = nextPe;
    nextPe = 0;
    pb = curP;  // 交换缓冲区
    curP = nextP;
    nextP = pb;

    // 查看是否完成了这个优先级级别
    if (curPe == 0) {
      curT += priInc;  // 增加优先级阈值
      curPe = overPe;  // 设置当前为溢出块
      overPe = 0;
      pb = curP;  // 交换缓冲区
      curP = overP;
      overP = pb;
    }

    // 检查是否到达了起始单元格
    if (potarr[startCell] < POT_HIGH) {
      break;
    }
  }

  last_path_cost_ = potarr[startCell];

  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    "[NavFn] 使用了 %d 个周期，%d 个单元格被访问 (%d%%)，优先级缓冲区最大值 %d\n",
    cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

  if (potarr[startCell] < POT_HIGH) {
    return true;  // 在这里完成
  } else {
    return false;
  }
}


float NavFn::getLastPathCost()
{
  return last_path_cost_;
}


//
// 路径构造
// 在数组点处找到梯度，插值路径
// 使用 pathStep 的步长，通常是 0.5 像素
//
// 一些健全性检查：
//  1. 卡在同一索引位置
//  2. 没有接近目标
//  3. 被高势能包围
//

int
NavFn::calcPath(int n, int * st)
{
  // 测试写入
  // savemap("test");

  // 检查路径数组
  if (npathbuf < n) {
    if (pathx) {delete[] pathx;}
    if (pathy) {delete[] pathy;}
    pathx = new float[n];
    pathy = new float[n];
    npathbuf = n;
  }

  // 设置起始位置在单元格
  // st 始终是左上角，用于 4 点双线性插值
  if (st == NULL) {st = start;}
  int stc = st[1] * nx + st[0];

  // 设置偏移量
  float dx = 0;
  float dy = 0;
  npath = 0;

  // 最多进行 <n> 个周期
  for (int i = 0; i < n; i++) {
    // 检查是否接近目标
    int nearest_point = std::max(
      0,
      std::min(
        nx * ny - 1, stc + static_cast<int>(round(dx)) +
        static_cast<int>(nx * round(dy))));
    if (potarr[nearest_point] < COST_NEUTRAL) {
      pathx[npath] = static_cast<float>(goal[0]);
      pathy[npath] = static_cast<float>(goal[1]);
      return ++npath;  // 完成！
    }

    if (stc < nx || stc > ns - nx) {  // 可能会超出边界
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] 超出边界");
      return 0;
    }

    // 添加到路径
    pathx[npath] = stc % nx + dx;
    pathy[npath] = stc / nx + dy;
    npath++;

    bool oscillation_detected = false;
    if (npath > 2 &&
      pathx[npath - 1] == pathx[npath - 3] &&
      pathy[npath - 1] == pathy[npath - 3])
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[PathCalc] 检测到振荡，尝试修复。");
      oscillation_detected = true;
    }

    int stcnx = stc + nx;
    int stcpx = stc - nx;

    // 检查单元格周围的八个位置的势能
  if (potarr[stc] >= POT_HIGH ||
      potarr[stc + 1] >= POT_HIGH ||
      potarr[stc - 1] >= POT_HIGH ||
      potarr[stcnx] >= POT_HIGH ||
      potarr[stcnx + 1] >= POT_HIGH ||
      potarr[stcnx - 1] >= POT_HIGH ||
      potarr[stcpx] >= POT_HIGH ||
      potarr[stcpx + 1] >= POT_HIGH ||
      potarr[stcpx - 1] >= POT_HIGH ||
      potarr[stc - nx - 1] >= POT_HIGH ||  // 左上角
      potarr[stc - nx + 1] >= POT_HIGH ||  // 右上角
      potarr[stc + nx - 1] >= POT_HIGH ||  // 左下角
      potarr[stc + nx + 1] >= POT_HIGH ||  // 右下角
      oscillation_detected)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] 势能边界，跟随网格 (%0.1f/%d)", potarr[stc], npath);

      // 检查八个邻居以找到最低的
      int minc = stc;
      int minp = potarr[stc];
      int st = stcpx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc + 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stcnx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      stc = minc;
      dx = 0;
      dy = 0;

      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "[Path] 势能: %0.1f  位置: %0.1f,%0.1f",
        potarr[stc], pathx[npath - 1], pathy[npath - 1]);

      if (potarr[stc] >= POT_HIGH) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] 没有找到路径，高势能");
        // savemap("navfn_highpot");
        return 0;
      }
    } else {  // 这里有一个好的梯度
      // 在四个位置附近获取梯度
      gradCell(stc);
      gradCell(stc + 1);
      gradCell(stc - 1);
      gradCell(stcnx);
      gradCell(stcnx + 1);
      gradCell(stcnx - 1);
      gradCell(stcpx);
      gradCell(stcpx + 1);
      gradCell(stcpx - 1);

      // 获取插值梯度
      float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
      float x2 = (1.0 - dx) * gradx[stcnx] + dx * gradx[stcnx + 1];
      float x = (1.0 - dy) * x1 + dy * x2;  // 插值 x
      float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
      float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
      float y = (1.0 - dy) * y1 + dy * y2;  // 插值 y

#if 0
      // 显示梯度
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; 最终 x=%.3f, y=%.3f\n",
        gradx[stc], grady[stc], gradx[stc + 1], grady[stc + 1],
        gradx[stcnx], grady[stcnx], gradx[stcnx + 1], grady[stcnx + 1],
        x, y);
#endif

      // 检查零梯度，失败
      if (x == 0.0 && y == 0.0) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] 零梯度");
        return 0;
      }

      // 向正确的方向移动
      float ss = pathStep / hypot(x, y);
      dx += x * ss;
      dy += y * ss;

      // 检查溢出
      if (dx > 1.0) {stc++; dx -= 1.0;}
      if (dx < -1.0) {stc--; dx += 1.0;}
      if (dy > 1.0) {stc += nx; dy -= 1.0;}
      if (dy < -1.0) {stc -= nx; dy += 1.0;}
    }

    //      ROS_INFO("[Path] 势能: %0.1f  梯度: %0.1f,%0.1f  位置: %0.1f,%0.1f\n",
    //      potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
  }

  //  return npath;  // 超出周期，返回失败
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] 没有找到路径，路径太长");
  // savemap("navfn_pathlong");
  return 0;  // 超出周期，返回失败
}


//
// 梯度计算
//

// 计算单元格的梯度
// 正值向右和向下
float
NavFn::gradCell(int n)
{
  if (gradx[n] + grady[n] > 0.0) {  // 检查此单元格
    return 1.0;
  }

  if (n < nx || n > ns - nx) {  // 可能会超出边界
    return 0.0;
  }

  float cv = potarr[n];
  float dx = 0.0;
  float dy = 0.0;

  // 检查是否在障碍物中
  if (cv >= POT_HIGH) {
      if (potarr[n - 1] < POT_HIGH) {
          dx = -COST_OBS;
      } else if (potarr[n + 1] < POT_HIGH) {
          dx = COST_OBS;
      }
      if (potarr[n - nx] < POT_HIGH) {
          dy = -COST_OBS;
      } else if (potarr[n + nx] < POT_HIGH) {
          dy = COST_OBS;
      }
      if (potarr[n - nx - 1] < POT_HIGH) {
          dx -= COST_OBS;
          dy -= COST_OBS;
      }
      if (potarr[n - nx + 1] < POT_HIGH) {
          dx += COST_OBS;
          dy -= COST_OBS;
      }
      if (potarr[n + nx - 1] < POT_HIGH) {
          dx -= COST_OBS;
          dy += COST_OBS;
      }
      if (potarr[n + nx + 1] < POT_HIGH) {
          dx += COST_OBS;
          dy += COST_OBS;
      }
  } else {  // 不在障碍物中
      // dx 计算，平均到两侧
      if (potarr[n - 1] < POT_HIGH) {
          dx += potarr[n - 1] - cv;
      }
      if (potarr[n + 1] < POT_HIGH) {
          dx += cv - potarr[n + 1];
      }
      if (potarr[n - nx - 1] < POT_HIGH) {
          dx += potarr[n - nx - 1] - cv;
          dy += potarr[n - nx - 1] - cv;
      }
      if (potarr[n - nx + 1] < POT_HIGH) {
          dx += cv - potarr[n - nx + 1];
          dy += potarr[n - nx + 1] - cv;
      }
      if (potarr[n + nx - 1] < POT_HIGH) {
          dx += potarr[n + nx - 1] - cv;
          dy += cv - potarr[n + nx - 1];
      }
      if (potarr[n + nx + 1] < POT_HIGH) {
          dx += cv - potarr[n + nx + 1];
          dy += cv - potarr[n + nx + 1];
      }

      // dy 计算，平均到两侧
      if (potarr[n - nx] < POT_HIGH) {
          dy += potarr[n - nx] - cv;
      }
      if (potarr[n + nx] < POT_HIGH) {
          dy += cv - potarr[n + nx];
      }
  }
  // 归一化
  float norm = hypot(dx, dy);
  if (norm > 0) {
    norm = 1.0 / norm;
    gradx[n] = norm * dx;
    grady[n] = norm * dy;
  }
  return norm;
}

}  // namespace nav2_navfn_planner