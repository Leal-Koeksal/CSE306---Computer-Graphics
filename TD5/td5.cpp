#define _CRT_SECURE_NO_WARNINGS 1

#include <algorithm>
#include <random>
#include <vector>
#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

static std::default_random_engine engine(10);
static std::uniform_real_distribution<double> uniform(0.0, 1.0);

#define ITERATIONS 100
#define M_PI 3.14159265359

class Vector {
public:
    explicit Vector(double x = 0, double y = 0, double z = 0) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }

    double norm2() const {
        return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
    }
    double norm() const {
        return sqrt(norm2());
    }
    void normalize() {
        double n = norm();
        data[0] /= n;
        data[1] /= n;
        data[2] /= n;
    }

    void operator+=(const Vector& b) {
        data[0] += b[0];
        data[1] += b[1];
        data[2] += b[2];
    }

    double operator[](int i) const { return data[i]; };
    double& operator[](int i) { return data[i]; };
    double data[3];
};

Vector operator+(const Vector& a, const Vector& b) {
    return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator-(const Vector& a) {
    return Vector(-a[0], -a[1], -a[2]);
}
Vector operator*(const double a, const Vector& b) {
    return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
    return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator/(const Vector& a, const double b) {
    return Vector(a[0] / b, a[1] / b, a[2] / b);
}
Vector cross(const Vector& a, const Vector& b) {
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}
double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector operator*(const Vector& a, const Vector& b) {
    return Vector(a[0]*b[0], a[1]*b[1], a[2]*b[2]);
}


Vector random_direction() {
    double r1 = uniform(engine);
    double r2 = uniform(engine);
    double x = cos(2 * M_PI * r1) * sqrt(r2 * (1 - r2));
    double y = sin(2 * M_PI * r1) * sqrt(r2 * (1 - r2));
    double z = 1 - 2 * r2;    
    Vector res = Vector(x, y, z);
    return res;
}

void color_matching(int W, int H, int C, int M_C, unsigned char* imageSource, unsigned char* colorSource) {
    int n = W * H; // total number of pixels
    std::vector<Vector> I(n), M(n);
    

    for (int i = 0; i < n; ++i) {
        I[i] = Vector(imageSource[C*i] / 255.0, imageSource[C*i + 1] / 255.0, imageSource[C*i + 2] / 255.0);
        M[i] = Vector(colorSource[M_C*i] / 255.0, colorSource[M_C*i + 1] / 255.0, colorSource[M_C*i + 2] / 255.0);
    }

    std::srand(std::time(0)); // ensure different sequence of numbers each run, used AI to write this line

    for (int iter = 0; iter < ITERATIONS; ++iter) {
        Vector v = random_direction();
        std::vector<std::pair<double, int>> projI(n), projM(n);

        for (int i = 0; i < n; ++i) {
            projI[i] = std::make_pair(dot(I[i], v), i);
            projM[i] = std::make_pair(dot(M[i], v), i);
        }

        std::sort(projI.begin(), projI.end());
        std::sort(projM.begin(), projM.end());

        for (int i = 0; i < n; ++i) {
            int idx = projI[i].second;
            double delta = projM[i].first - projI[i].first;
            I[idx] += v * delta;
        }
    }

    for (int i = 0; i < n; ++i) {
        imageSource[C*i]     = std::min(255, std::max(0, int(I[i][0] * 255.0)));
        imageSource[C*i + 1] = std::min(255, std::max(0, int(I[i][1] * 255.0)));
        imageSource[C*i + 2] = std::min(255, std::max(0, int(I[i][2] * 255.0)));
    }
}

int main(int argc, char **argv) {
    // input image
	int inputw;
    int inputh;
    int inputc;
    // model image
	int modelw;
    int modelh;
    int modelc;

	unsigned char *imageSource = stbi_load("input.png", &inputw, &inputh, &inputc, 0);
	unsigned char *colorSource = stbi_load("model.png", &modelw, &modelh, &modelc, 0);

	color_matching(inputw, inputh, inputc, modelc, imageSource, colorSource);

	stbi_write_png("result.png", inputw, inputh, inputc, &imageSource[0], 0);

	return 0;
}
