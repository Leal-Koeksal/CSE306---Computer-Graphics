#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define M_PI 3.14159265358979323846
#include <cmath> 
#include <iostream>
#include <algorithm>
#include <limits>
#include <random>
#include <chrono>

static std::default_random_engine engine(10);
static std::uniform_real_distribution<double> uniform(0.0, 1.0);


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
Vector operator*(const double a, const Vector& b) {
    return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
    return Vector(a[0]*b, a[1]*b, a[2]*b);
}

// my addition
Vector operator*(const Vector& a, const Vector& b) {
    return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}

Vector operator/(const Vector& a, const double b) {
    return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

// sampling for indirect lighting
Vector random_cos(Vector& N) {
    Vector V;

    double r1 = (float)rand() / RAND_MAX;
    double r2 = (float)rand() / RAND_MAX;

    double x = sqrt(1 - r2) * cos(2 * M_PI * r1);
    double y = sqrt(1 - r2) * sin(2 * M_PI * r1);
    double z = sqrt(r2);

    double min = std::abs(N[0]);
    int id = 0;
    for (int i = 1; i < 3; ++i) {
        if (std::abs(N[i]) < min ) {
            min = std::abs(N[i]);
            id = i;
        }
    }

    Vector T1 = -1 * N;
    T1[id] = 0;
    int id1 = 3 - id;
    int id2 = 3 - id - id1;
    std::swap(T1[id1], T1[id2]);

    Vector T2 = cross(N, T1);

    return x * T1 + y * T2 + z * N;

}

double sqr (double x) {
    return x * x;
}

class Ray {
    public:
        Ray(const Vector& O, const Vector& u) : O(O), u(u) {};
        Vector O;
        Vector u;
};

class Sphere {
public:
    Sphere(const Vector& C, double R, const Vector& rho, bool is_mirror = false, bool is_transparent = false, double n = 1.0) 
    : C(C), R(R), rho(rho), mirror(is_mirror), transparent(is_transparent), n(n) {};

    Vector C;
    double R;
    Vector rho;
    bool mirror;
    bool transparent;
    double n;
    
    bool intersect(const Ray& r, Vector &P , Vector &N, bool& is_in) {
        double delta = sqr(dot(r.u, r.O - C)) - ((r.O - C).norm2() - sqr(R));  // the discriminant

        // if delta < 0, there is no intersection
        if (delta < 0) {
            return false;
        }

        double x = dot(r.u, C - r.O);
        double t1 = x - sqrt(delta);
        double t2 = x + sqrt(delta);
        double t;
        
        if (t2 < 0) {
            return false;
        }  

        if (t1 > 0) {
            t = t1;
        }
        else {
            t = t2;
        }
     
        P = r.O + t * r.u;

        N = (P - C) / (P - C).norm();

        return true;
    }
};

class Public {
    public:
};

class Scene {
    public:
        std::vector<Sphere> spheres;

        void add(const Sphere& s) {
            spheres.push_back(s);
        }

        bool intersect(const Ray& r, Vector& P, Vector& N, int &id, Vector& rho, bool& is_in) { // t is the point we are looking for
            bool result = false;
            double closest_t = std::numeric_limits<double>::max();

            for (size_t i = 0; i < spheres.size(); ++i) {
                Vector tempP;
                Vector tempN;

                if (spheres[i].intersect(r, tempP, tempN, is_in)) {
                    double t = (tempP - r.O).norm();
                    if (t < closest_t) {
                        P = tempP;
                        N = tempN;
                        rho = spheres[i].rho;
                        closest_t = t;
                        id = i;
                        result = true;
                    }
                }
                
            }

            return result;
        }

        Vector getColour(const Ray& ray, int depth, Vector lpos, double I, bool& is_in) {
            // Base Case
            if (depth <= 0) {
                return Vector(0, 0, 0);
            }
            Vector colour = Vector(0, 0, 0);

            double eps = 1e-5;

            Vector P;
            Vector N;
            Vector rho; // rho of the hit sphere
            int id;

            if(intersect(ray, P, N, id, rho, is_in)) {
                const Sphere& S = spheres[id];
                is_in = dot(ray.u, N) < 0 ? false : true;

                if (S.mirror) {
                    Vector refl_dir = ray.u - N * (2 * dot(ray.u, N));
                    refl_dir.normalize();
                    return getColour(Ray(P + N * eps, refl_dir), depth - 1, lpos, I, is_in);
                }

                else if (S.transparent) {
                    // incident ray
                    Vector i = ray.u; 
                    i.normalize();
                    double Ni = dot(N, i);

                    // fresnel reflection -- Bruno Iorio helped me with the logic here :)

                    double n1 = 1.0; // refarction index for air
                    double n2 = S.n; // refraction index for sphere
                    bool entering = Ni < 0;

                    if (!entering) {
                        std::swap(n1, n2);
                        N = -1 * N;
                        Ni = dot(N, i);
                    }

                    Vector refl_origin = P + N * eps;

                    double ratio = n1 / n2;
                    double c = 1 - sqr(ratio) * (1 - sqr(Ni));

                    if (c < 0) { 
                        // total internal reflection
                        Vector refl_dir = i - 2 * Ni * N;
                        refl_dir.normalize();
                        Ray refl = Ray(refl_origin, refl_dir);
                        return getColour(refl, depth - 1, lpos, I, is_in);
                    }
                    else {
                        // refraction -- apply Fresnel
                        Vector vt = ratio * (i - Ni * N);
                        Vector vn = -1 * N * sqrt(c);
                        Vector vf = vt + vn;
                        vf.normalize();
                        Vector refr_origin = P - N * eps;

                        double k0 = sqr(n1 - n2) / sqr(n1 + n2);
                        double R = k0 + (1 - k0) * pow((1 - std::abs(Ni)), 5); 
                        double T = 1 - R;

                        Ray refr = Ray(refr_origin, vf);
                        Vector refr_colour = getColour(refr, depth - 1, lpos, I, is_in);

                        Vector refl_dir = i - 2 * Ni * N;
                        refl_dir.normalize();
                        Ray refl = Ray(refl_origin, refl_dir);
                        Vector refl_colour = getColour(refl, depth - 1, lpos, I, is_in);
                        
                        return refl_colour * R + refr_colour * T;

                    }
                }

                else {
                    // check for shadow
                    if (isShadow(P, N, lpos, is_in)) {
                        return colour;
                    }

                    else {
                        // direct lighting
                        Vector L = lpos - P;
                        double dist = L.norm2();
                        Vector material = rho / M_PI;
                        double const1 = I / (4 * M_PI * dist);
                        double pos_dot = std::max(0.0, dot(N, L / L.norm()));
                        colour = colour + material * const1 * pos_dot;

                        // indirect lighting
                        Vector random_dir = random_cos(N);
                        Ray random_ray = Ray(P, random_dir);
                        colour = colour + (rho * getColour(random_ray, depth - 1, lpos, I, is_in));
                        return colour;
                        
                    }

                }
            }

            return colour;
        }

        bool isShadow(const Vector& P, const Vector& N, const Vector& lpos, bool& is_in) {
            Vector ldir = lpos - P;
            double ldist = ldir.norm();
            ldir.normalize();

            Ray shadowRay = Ray(P + N * 1e-4, ldir);
            Vector tempP;
            Vector tempN;
            int id;
            Vector rho;

            if (intersect(shadowRay, tempP, tempN, id, rho, is_in)) {
                double hdist = (tempP - P).norm();
                if (hdist < ldist) {
                    // Blocking the light
                    return true;
                }
            }

            // No intersection, the light is visible
            return false;
        }


};

// this function is from the lecture notes
void boxMuller(double stdev, double &x, double &y) {
    double r1 = uniform(engine);
    double r2 = uniform(engine);
    x = sqrt(-2 * log(r1)) * cos(2 * M_PI * r2) * stdev;
    y = sqrt(-2 * log(r1)) * sin(2 * M_PI * r2) * stdev;
}


int main() {
    auto start = std::chrono::high_resolution_clock::now();

    int W = 512;
    int H = 512;
    int max_depth = 50;
    int rpp = 1; // rays per pixel
    Scene scene;

    // Subjects of the Image
    scene.add(Sphere(Vector(-20, -1, 0), 9, Vector(0.7, 0.1, 0.5), false, true, 1.5));
    scene.add(Sphere(Vector(0, -1, 0), 9, Vector(0.7, 0.1, 0.5), true));
    scene.add(Sphere(Vector(20, -1, 0), 9, Vector(0.7, 0.1, 0.5)));

    // Background
    scene.add(Sphere(Vector(0,0,1000), 940, Vector(0.9,0.2,0.9))); // left-wall
    scene.add(Sphere(Vector(0,0,-1000), 940, Vector(0.6,0.5,0.1))); // right-wall
    scene.add(Sphere(Vector(0,-1000,0), 990, Vector(0.3,0.4,0.7))); // floor
    scene.add(Sphere(Vector(0,1000,0), 940, Vector(0.2,0.5,0.9))); // ceiling
    scene.add(Sphere(Vector(-1000,0,0), 940, Vector(0.4,0.8,0.7))); // back-wall
    scene.add(Sphere(Vector(1000,0,0), 940, Vector(0.9,0.4,0.3))); // other wall

    // Scene set-up
    std::vector<unsigned char> image(W * H * 3, 0);
    Vector camerao = Vector(0, 0, 55);
    double fieldofview = 60 * M_PI / 180;

    // Light settings
    double I = 1E5;
    // Vector lpos = Vector(-10, 20, 40);
    Vector lpos = Vector(-10, 25, -10);
    // scene.add(Sphere(lpos, 5, Vector(1.0, 1.0, 1.0))); // Light source as a sphere

    // Scan all pixels
    bool flag = false;

    #pragma omp parallel for schedule(dynamic, 1) // parallelises code
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {

            Vector colour = Vector(0, 0, 0);
            double x, y;

            for (int k = 0; k < rpp; ++k) {
                boxMuller(0.5, x, y);
                Vector pixel = Vector(camerao[0] + (j+x) + 0.5 - W/2,
                                      camerao[1] - (i+y) - 0.5 + H/2,
                                      camerao[2] - W/(2 * tan(fieldofview/2))
                                );
                Vector ray_dir = pixel - camerao;
                ray_dir.normalize();
                Ray ray = Ray(camerao, ray_dir);
                colour = colour + scene.getColour(ray, max_depth, lpos, I, flag);
            }

            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(colour[0] / rpp, 1 / 2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(colour[1] / rpp, 1 / 2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(colour[2] / rpp, 1 / 2.2));
        }

    }

    stbi_write_png("test.png", W, H, 3, &image[0], 0);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
    std::cout << "Rendering took " << duration.count() << " milliseconds." << std::endl;
 
    return 0;
}


