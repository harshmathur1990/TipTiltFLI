// min_element/max_element example
#include <iostream>     // std::cout
#include <algorithm>    // std::min_element, std::max_element

int main () {
    int myints[] = {3,10,2,5,6,4,9};

    std::cout<< std::distance(myints, std::max_element(myints, myints+7))<<std::endl;

    int a;
    std::cin>>a;
    return 0;
}

