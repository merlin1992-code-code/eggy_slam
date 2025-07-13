/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-13 16:12:40
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-13 22:17:44
 */
#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <yaml-cpp/yaml.h>

struct LidarParam {
    std::vector<float> unprj_row_angles_const;
    std::vector<float> unprj_col_angles_const;
    float sensor_height;
    float reverse_step_row_angle;
    float half_row_angle_step;
    int row_angles_size;
    int first_index;
    int last_rows_index;
    float reverse_step_col_angle;
    float half_col_angle_step;
    int col_angles_size;
    int last_cols_index;
    float cpst_d_th;
    std::vector<int> cpst;
    float close_region_boundary_x_neg;
    float close_region_boundary_x_pos;
    float close_region_boundary_y;
    std::string ground_pcd_dir;
};

// 生成水平角度数组
std::vector<float> generate_unprj_col_angles(int col_angles_size) {
    std::vector<float> angles(col_angles_size);
    float step = 2 * M_PI / col_angles_size;
    for (int i = 0; i < col_angles_size; ++i) {
        angles[i] = -M_PI + i * step;
    }
    return angles;
}

// 从dipg-seg 论文中生成cpst
std::vector<int> generate_cpst_from_paper(const std::vector<float>& row_angles, float z_sensor, float d_thr) {
    std::cout << "generate_cpst_from_paper" << std::endl;
    std::vector<int> cpst;
    int NR = row_angles.size();

    for (int l = 0; l < NR; ++l) {
        float sum_dd = 0;
        int i = 0;
        while (sum_dd <= d_thr) {
            i++;
            if (l + i >= NR) break; // 防止越界
            float tan1 = std::tan(row_angles[l]);
            float tan2 = std::tan(row_angles[l + i]);
            if (std::abs(tan1 - tan2) < 1e-6) break; // 防止除零
            float dd = z_sensor / tan1 - z_sensor / tan2;
            sum_dd += std::abs(dd);
        }
        cpst.push_back(i);
    }
    return cpst;
}

bool load_lidar_param(const std::string& yaml_path, LidarParam& param) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        param.unprj_row_angles_const = config["row_angles"].as<std::vector<float>>();
        std::reverse(param.unprj_row_angles_const.begin(), param.unprj_row_angles_const.end());
        param.sensor_height = config["sensor_height"].as<float>();
        param.reverse_step_row_angle = config["reverse_step_row_angle"].as<float>();
        param.half_row_angle_step = config["half_row_angle_step"].as<float>();
        param.row_angles_size = config["row_angles_size"].as<int>();
        param.first_index = config["first_index"].as<int>();
        param.last_rows_index = config["last_rows_index"].as<int>();
        param.reverse_step_col_angle = config["reverse_step_col_angle"].as<float>();
        param.half_col_angle_step = config["half_col_angle_step"].as<float>();
        param.col_angles_size = config["col_angles_size"].as<int>();
        param.last_cols_index = config["last_cols_index"].as<int>();
        param.cpst_d_th = config["cpst_d_th"].as<float>();
        param.close_region_boundary_x_neg = config["close_region_boundary_x_neg"].as<float>();
        param.close_region_boundary_x_pos = config["close_region_boundary_x_pos"].as<float>();
        param.close_region_boundary_y = config["close_region_boundary_y"].as<float>();

        param.unprj_col_angles_const = generate_unprj_col_angles(param.col_angles_size);
        param.cpst = generate_cpst_from_paper(param.unprj_row_angles_const, param.sensor_height, param.cpst_d_th);
        //param.cpst = config["cpst"].as<std::vector<int>>();
        param.ground_pcd_dir = config["ground_pcd_dir"].as<std::string>();
        return true;
    } catch (std::exception& e) {
        std::cerr << "Failed to load lidar param yaml: " << e.what() << std::endl;
        return false;
    }
}

