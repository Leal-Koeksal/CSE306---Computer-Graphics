#define _CRT_SECURE_NO_WARNINGS 1

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define M_PI 3.14159265358979323846
#include <algorithm>
#include <chrono>
#include <cmath> 
#include <iostream>
#include <limits>
#include <random>
#include <stdio.h>
#include <string>
#include <vector>

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
 
class TriangleIndices {
public:
    TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
    };
    int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
    int uvi, uvj, uvk;  // indices within the uv coordinates array
    int ni, nj, nk;  // indices within the normals array
    int group;       // face group
};
 
 
class TriangleMesh {
public:
  ~TriangleMesh() {}
    TriangleMesh() {};
    
    void readOBJ(const char* obj) {
 
        char matfile[255];
        char grp[255];
 
        FILE* f;
        f = fopen(obj, "r");
        int curGroup = -1;
        while (!feof(f)) {
            char line[255];
            if (!fgets(line, 255, f)) break;
 
            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());
 
            if (line[0] == 'u' && line[1] == 's') {
                sscanf(line, "usemtl %[^\n]\n", grp);
                curGroup++;
            }
 
            if (line[0] == 'v' && line[1] == ' ') {
                Vector vec;
 
                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
                    col[0] = std::min(1., std::max(0., col[0]));
                    col[1] = std::min(1., std::max(0., col[1]));
                    col[2] = std::min(1., std::max(0., col[2]));
 
                    vertices.push_back(vec);
                    vertexcolors.push_back(col);
 
                } else {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n') {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't') {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f') {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;
                t.group = curGroup;
 
                char* consumedline = line + 1;
                int offset;
 
                nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9) {
                    if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                    if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                    if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                    if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                    if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                    if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                    if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                    if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                    if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                    indices.push_back(t);
                } else {
                    nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6) {
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                        if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                        if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                        if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                        indices.push_back(t);
                    } else {
                        nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                        if (nn == 3) {
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            indices.push_back(t);
                        } else {
                            nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                            if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                            if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }
 
                consumedline = consumedline + offset;
 
                while (true) {
                    if (consumedline[0] == '\n') break;
                    if (consumedline[0] == '\0') break;
                    nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.group = curGroup;
                    if (nn == 3) {
                        if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                        if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                        if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                        if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                        if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                        if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                        if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                        if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                        if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    } else {
                        nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                        if (nn == 2) {
                            if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                            if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                            if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                            if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                            if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                            if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        } else {
                            nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                            if (nn == 2) {
                                if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                                if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                                if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;                             
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            } else {
                                nn = sscanf(consumedline, "%u%n", &i3, &offset);
                                if (nn == 1) {
                                    if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                    if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                    if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                } else {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }
 
            }
 
        }
        fclose(f);
 
    }
 
    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> vertexcolors;
    
};

class Objects {
public:

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

class Intersection {
public:
    Intersection(const Vector& P, const Vector& N, const Vector& rho, const int& id) 
    : P(P), N(N), rho(rho), id(id) {};

    Vector P;
    Vector N;
    Vector rho; // albedo
    int id;

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
    int max_depth = 10;
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
    double I = 1.2E5;
    Vector lpos = Vector(-10, 20, 40);
    // Vector lpos = Vector(-10, 25, -10);
    
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

            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(colour[0] / rpp, 1 / 2.2) * 255);
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(colour[1] / rpp, 1 / 2.2) * 255);
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(colour[2] / rpp, 1 / 2.2) * 255);
        }

    }

    stbi_write_png("test.png", W, H, 3, &image[0], 0);

    auto stop = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
    std::cout << "Time: " << time.count() << " milliseconds." << std::endl;
 
    return 0;
}
