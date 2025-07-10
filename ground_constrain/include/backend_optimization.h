/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-10 11:20:30
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-10 11:20:32
 */
#ifndef BACKEND_OPTIMIZATION_H
#define BACKEND_OPTIMIZATION_H

#pragma once
#include <vector>
#include "KeyFrame.h"

// 对关键帧序列进行地面约束优化，结果回写到 keyframes[i].pose
void optimizeWithGroundConstraint(std::vector<KeyFrame>& keyframes);

#endif // BACKEND_OPTIMIZATION_H