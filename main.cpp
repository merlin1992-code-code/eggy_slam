/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-28 20:43:56
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-06-28 21:15:26
 */
#include <iostream>
#include "fastlio/lio_node.h"
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " config.yaml" << std::endl;
        return 1;
    }

    // Initialize LIO Node
    auto lio_node = std::make_shared<LIONode>(argv[1]);
    lio_node->execute();

    return 0;
}