#include <pathfinder.h>

#include <iostream>
#include <functional>
#include <ctime>

using namespace Pathfinder;

#define SIZE 32

void timeit(std::function<void()> func) {
    std::clock_t start = std::clock();

    func();

    int ms = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);

    std::cout << "Finished in " << ms << "ms" << std::endl;
}

int main() {
    Vec2 v[SIZE] = { };
    for (int i = 0; i < SIZE; i++) {
        v[i].x = i*2;
        v[i].y = i*2+1;
    }

    Vec2Batch<SIZE> b(v);
    Vec2Batch<SIZE> simd_b;
    Vec2Batch<SIZE> nat_b;

    timeit([&simd_b, &b] { for (int i = 0; i < 100000; i++) simd_b = b*2; });
    timeit([&nat_b, &b] { for (int i = 0; i < 100000; i++) nat_b = b.mult(2); });

    for (int i = 0; i < SIZE; i++) {
        std::cout << (std::string) *b[i] << " ";
    }
    std::cout << "\n\n";
    for (int i = 0; i < SIZE; i++) {
        std::cout << (std::string) *simd_b[i] << " ";
    }
    std::cout << "\n";
    for (int i = 0; i < SIZE; i++) {
        std::cout << (std::string) *nat_b[i] << " ";
    }
    std::cout << "\n";
}