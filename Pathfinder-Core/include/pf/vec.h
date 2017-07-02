#pragma once

#include <cmath>
#include <string>
#include <string.h>

namespace Pathfinder {
    #pragma pack(push, 4)
    class Vec2 {
    public:
        Vec2() { }
        Vec2(float x, float y) {
            this->x = x;
            this->y = y;
        }

        Vec2 normal() {
            Vec2 v(-y, x);
            return v;
        }

        Vec2 unit() {
            float mag = magnitude();
            Vec2 v(x/mag, y/mag);
            return v;
        }

        float magnitude() {
            return sqrt(x*x + y*y);
        }

        float argument() {
            return atan2(y, x);
        }

        Vec2 operator+(Vec2 other) {
            Vec2 v(x + other.x, y + other.y);
            return v;
        }

        Vec2 operator-(Vec2 other) {
            Vec2 v(x - other.x, y - other.y);
            return v;
        }

        Vec2 operator-() {
            Vec2 v(-x, -y);
            return v;
        }

        Vec2 operator*(float scalar) {
            Vec2 v(x*scalar, y*scalar);
            return v;
        }

        // Dot Product
        float operator*(Vec2 other) {
            return x*other.x + y*other.y;
        }

        Vec2 operator/(float scalar) {
            Vec2 v(x/scalar, y/scalar);
            return v;
        }

        operator std::string() {
            char buf[256];
            sprintf(buf, "<%.2f, %.2f>", x, y);
            return std::string(buf);
        }

        static Vec2 polar(float magnitude, float argument) {
            Vec2 v(magnitude * cos(argument), magnitude * sin(argument));
            return v;
        }

        float x, y;
    };
    #pragma pack(pop)

    // Alternate Multiplication member of Vec2
    Vec2 operator*(float scalar, Vec2& vec) {
        return vec*scalar;
    }

    // A container for multiple Vec2 objects. These can be used in order
    // to batch process Vec2s using OpenMP SIMD on supporting platforms.
    template<size_t SIZE>
    class Vec2Batch {
    public:
        Vec2Batch() { }
        Vec2Batch(Vec2 *vec2s) {
            memcpy((void *)xy, (void *)vec2s, sizeof(xy));
        }
        Vec2Batch(const Vec2Batch<SIZE> &other) {
            memcpy((void *)other.xy, (void *)xy, sizeof(xy));
        }

        Vec2 *operator[](int index) {
            return (Vec2 *)&xy[index*2];
        }

        Vec2Batch<SIZE> operator*(float scalar) {
            Vec2Batch<SIZE> b;
            #pragma omp simd
            for (int i = 0; i < SIZE*2; i++) {
                b.xy[i] = xy[i] * scalar;
            }
            return b;
        }

        Vec2Batch<SIZE> mult(float scalar) {
            Vec2Batch<SIZE> b;
            for (int i = 0; i < SIZE*2; i++) {
                b.xy[i] = xy[i] * scalar;
            }
            return b;
        }

        float xy[SIZE*2];
    };
}