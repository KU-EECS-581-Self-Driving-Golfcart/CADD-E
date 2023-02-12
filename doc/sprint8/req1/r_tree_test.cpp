#include <iostream>

#include "r_tree.h"
#include "point.h"

#include <tuple>

int main() {
    R_Tree my_tree;

    int upper_bound = 7;

    for(int i = 0; i < upper_bound; i++) {
        for(int j = 0; j < upper_bound; j++) {
            Point p(i + 0.1*j, j + 0.1*i);
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
            my_tree.insert(p);
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
        }
    }

    my_tree.print();

    return 0;
}