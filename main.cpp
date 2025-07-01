/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-28 20:43:56
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-01 11:07:22
 */
#include <iostream>
#include <string>
#include "fastlio/lio_node.h"
#include "m-detector/dyn_node.h"

using namespace std;

int main(int argc, char **argv)
{
    std::string lio_config, dyn_config;
    for (int i = 1; i < argc - 1; ++i)
    {
        if (std::string(argv[i]) == "--lio")
            lio_config = argv[++i];
        else if (std::string(argv[i]) == "--dyn")
            dyn_config = argv[++i];
    }
    if (lio_config.empty() || dyn_config.empty())
    {
        std::cerr << "Usage: " << argv[0] << " --lio lio_config.yaml --dyn dyn_config.yaml" << std::endl;
        return 1;
    }

    auto lio_node = std::make_shared<LIONode>(lio_config);
    //lio_node->execute();

    auto dyn_node = std::make_shared<DynNode>(dyn_config);
    dyn_node->execute();

    return 0;
}