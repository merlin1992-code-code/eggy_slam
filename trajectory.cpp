#include "trajectory.h"
#include <fstream>
#include <sstream>

bool Trajectory::saveToCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    file << "time,x,y,z,rot00,rot01,rot02,rot10,rot11,rot12,rot20,rot21,rot22\n";
    for (const auto& p : poses) {
        file << p.time << "," << p.pos.x() << "," << p.pos.y() << "," << p.pos.z();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                file << "," << p.rot(i, j);
        file << "\n";
    }
    return true;
}

bool Trajectory::loadFromCSV(const std::string& filename) {
    poses.clear();
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    std::string line;
    std::getline(file, line); // skip header
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<double> vals;
        while (std::getline(ss, item, ',')) {
            vals.push_back(std::stod(item));
        }
        if (vals.size() == 13) {
            Eigen::Vector3d pos(vals[1], vals[2], vals[3]);
            Eigen::Matrix3d rot;
            rot << vals[4], vals[5], vals[6],
                   vals[7], vals[8], vals[9],
                   vals[10], vals[11], vals[12];
            poses.push_back({vals[0], pos, rot});
        }
    }
    return true;
}