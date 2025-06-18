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
Vector operator/(const Vector& a, const double b) {
    return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
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


int main() {
// #pragma omp parallel for schedule(dynamic, 1) // parallelises code
    int W = 512;
    int H = 512;
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
    double I = 1E8;
    Vector lpos = Vector(-10, 20, 40);

    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            
            // Find light ray
            double z = -W / (2 * tan(fieldofview / 2));
            Vector r_dir(j - W/2 + 0.5, H/2 -i + 0.5, z); // 0.5 is to throw ray to center of pixel
            r_dir.normalize();
            Ray r(camerao, r_dir);
            bool flag = false;

            Vector colour = scene.getColour(r, 20, lpos, I, flag);

            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(colour[0] * 255, 1 / 2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(colour[1] * 255, 1 / 2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(colour[2] * 255, 1 / 2.2));
        }

    }

    stbi_write_png("TD1res.png", W, H, 3, &image[0], 0);
 
    return 0;
}


