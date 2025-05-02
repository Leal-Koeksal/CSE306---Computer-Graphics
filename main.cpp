

/*
DISCLAIMER
I used the following github to debug my code when I got stuck. I did my best not to copy the ideas or code, but only compare snippets of code that were not working 
for me at all. Great thanks to this student who seems to have done a great job!
https://github.com/vrushank-agrawal/CSE306/tree/7ae1812edfc369827a417d57afe801f7b93f0b9b
*/


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
#include <list>

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

    double r1 = uniform(engine);
    double r2 = uniform(engine);

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

    Vector T1;
    if (id == 0) T1 = Vector(0, N[2], -N[1]);
    else if (id == 1) T1 = Vector(N[2], 0, -N[0]);
    else T1 = Vector(N[1], -N[0], 0);
    T1.normalize();

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

struct Intersection {
    public:
        // default constructor 
        Intersection(Vector rho = Vector(0, 0, 0), double n = 1, bool is_mirror = false, bool is_transparent = false)
        : rho(rho), n(n), mirror(is_mirror), transparent(is_transparent) {}

        bool intersects;
        Vector P; // point of intersection
        Vector N; // normal
        Vector rho; // albedo
        int id; // id of sphere
        double t;

        double n; // refractive index
        bool mirror; 
        bool transparent;

};

class Geometry {
    public:
        // default constrcutor
        Geometry(const Vector& rho = Vector(1, 1, 1), bool mirror = false, bool transparent = false, double n = 1.0)
        : rho(rho), mirror(mirror), transparent(transparent), n(n) {}

        virtual ~Geometry() {}

        // prototype to indicate implementation is not provided
        virtual Intersection intersect(const Ray& r, bool& is_in) const = 0;

        Vector rho;
        bool mirror;
        bool transparent;
        double n;
    };

class BoundingBox {
public:
    BoundingBox(Vector bmin = Vector(), Vector bmax = Vector()) 
    : bmin(bmin), bmax(bmax) {}

    Vector bmin;
    Vector bmax;

    void expand(const Vector& P) {
        for (int i = 0; i < 3; ++i) {
            bmin[i] = std::min(bmin[i], P[i]);
            bmax[i] = std::max(bmax[i], P[i]);
        }
    }

    bool intersect(const Ray& r, double& t) const { 
        double tx_min = (bmin[0] - r.O[0]) /r.u[0];
        double tx_max = (bmax[0] - r.O[0]) /r.u[0];
        if (tx_min > tx_max) std::swap(tx_min, tx_max);

        double ty_min = (bmin[1] - r.O[1]) /r.u[1];
        double ty_max = (bmax[1] - r.O[1]) /r.u[1];
        if (ty_min > ty_max) std::swap(ty_min, ty_max);

        double tz_min = (bmin[2] - r.O[2]) /r.u[2];
        double tz_max = (bmax[2] - r.O[2]) /r.u[2];
        if (tz_min > tz_max) std::swap(tz_min, tz_max);

        double t_enter = std::max({tx_min, ty_min, tz_min});
        double t_exit = std::min({tx_max, ty_max, tz_max});

        if (t_enter > t_exit || t_exit < 0) return false;
        t = t_enter;
        return true;
    }
};

class Node {
public:
    BoundingBox bbox;
    Node* left;
    Node* right;
    int start;
    int end;

    // constructor
    Node() : left(nullptr), right(nullptr), start(0), end(0) {}
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
 
class TriangleMesh : public Geometry {
private:
    double scaling_factor;
    Vector translation;
    Node* root;

    void delBVH(Node* node) {
        if (!node) return;
        delBVH(node->left);
        delBVH(node->right);
        delete node;
    }

    void buildBHV(Node* node, int start, int end, int depth = 0) {
        // std::cout << "BVH depth " << depth << ", tris: " << end - start << "\n"; // debug
        if (depth > 32) {
            std::cerr << "Warning: BVH depth exceeded 32 at node with " << end - start << " triangles\n";
            return;
        }
        
        node->bbox = computeBB(start, end);
        node->start = start;
        node->end = end;

        Vector d = node->bbox.bmax - node->bbox.bmin;
        Vector middle = node->bbox.bmin + d * 0.5;
        int axis = 0; // we want to find the longest axis
        if (d[1] > d[0] && d[1] > d[2]) axis = 1;
        if (d[2] > d[0] && d[2] > d[1]) axis = 2;

        int p = start;
        for (int i = start; i < end; ++i) {
            Vector b = computeBC(indices[i]);
            if (b[axis] < middle[axis]) {
                std::swap(indices[i], indices[p]);
                p++;
            }
        }

        // partition fails
        if (p <= start || p >= end - 1 || end - start < 5) return;


        node->left = new Node();
        node->right = new Node();
        this->buildBHV(node->left, start, p, depth + 1);
        this->buildBHV(node->right, p, end, depth + 1);
    }

public:
    ~TriangleMesh() {delBVH(root);}
    TriangleMesh(const double& scaling_factor, const Vector& translation, const Vector& rho, bool mirror = false, bool transparent = false, double n = 1.0) 
    : scaling_factor(scaling_factor), translation(translation), Geometry(rho, mirror, transparent, n) {};

    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> vertexcolors;

    BoundingBox bbox;

    void createBVH() {
        root = new Node();
        this->buildBHV(root, 0, indices.size());
    }

    // aux function
    BoundingBox computeBB(int start, int end) {
        Vector init = scaling_factor * vertices[indices[start].vtxi] + translation;
        BoundingBox tempbox = BoundingBox(init, init);
        for (int i = start; i < end; ++i) {
            Vector A = scaling_factor * vertices[indices[i].vtxi] + translation;
            Vector B = scaling_factor * vertices[indices[i].vtxj] + translation;
            Vector C = scaling_factor * vertices[indices[i].vtxk] + translation;
            tempbox.expand(A);
            tempbox.expand(B);
            tempbox.expand(C);
        }

        return tempbox;
    }

    //aux functions
    Vector computeBC(const TriangleIndices& t) {
        Vector v1 = scaling_factor * vertices[t.vtxi] + translation;
        Vector v2 = scaling_factor * vertices[t.vtxj] + translation;
        Vector v3 = scaling_factor * vertices[t.vtxk] + translation;
        return (v1 + v2 + v3) / 3.0;

    }

    Intersection intersect(const Ray& r, bool& is_in) const override {
        Intersection I = Intersection(this->rho, this->n, this->mirror, this->transparent);
        I.intersects = false;
        double closest_t = std::numeric_limits<double>::max();

        // check for intersection with bounding box
        double t_box;
        if (!root->bbox.intersect(r, t_box)) {
            return I;
        }

        std::list<Node*> bvh;
        bvh.push_back(root);

        while (!bvh.empty()) {
            Node* node = bvh.back();
            bvh.pop_back();

            if (!node->left && !node->right) {
                //debug
                int triCount = node->end - node->start;
                Vector e1, e2, N;
                for (int i = node->start; i < node->end; ++i) {
                    const Vector& A = scaling_factor * vertices[indices[i].vtxi] + translation; // add variables for scaling
                    const Vector& B = scaling_factor * vertices[indices[i].vtxj] + translation;
                    const Vector& C = scaling_factor * vertices[indices[i].vtxk] + translation;
        
                    e1 = B - A;
                    e2 = C - A;
                    N = cross(e1, e2);
        
                    Vector numer = cross(A - r.O, r.u);
                    double denom = dot(r.u, N);
        
                    double beta = dot(e2, numer) / denom;
                    double gamma = -1 * dot(e1, numer) / denom;
                    double alpha = 1 - beta - gamma;
                    double t = dot(A - r.O, N) / denom;
        
                    if (alpha >= 0.0 && alpha <= 1.0 
                        && beta >= 0.0 && beta <= 1.0 
                        && gamma >= 0.0 && gamma <= 1.0 
                        && t >= 0.0 && t < closest_t) {
        
                        closest_t = t;
                        I.intersects = true;
                        I.t = closest_t;
                        I.P = r.O + closest_t * r.u; 
                        N.normalize();
                        I.N = N;
                        I.id = i;
                    }
                }
            }
            else {
                double tl, tr;
                bool hl = node->left && node->left->bbox.intersect(r, tl);
                bool hr = node->right && node->right->bbox.intersect(r, tr);
                
                if (hl && tl < closest_t && node->left != node) bvh.push_back(node->left);
                if (hr && tr < closest_t && node->left != node) bvh.push_back(node->right);
            }
        }
        
        if (I.intersects) {
            is_in = dot(r.u, I.N) > 0;
            // maybe remove this
            if (is_in) {
                I.N = -1 * I.N;
            }
        }

        return I;
    }
    
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

                    // my modification
                    Vector transformed = scaling_factor * vec + translation;
                    bbox.expand(transformed);
 
                } else {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);

                    // my modification
                    Vector transformed = scaling_factor * vec + translation;
                    bbox.expand(transformed);


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
        // my modification
        this->buildBHV(root, 0, indices.size());
 
    }

};

class Sphere : public Geometry {
public:
    Sphere(const Vector& C, double R, const Vector& rho, bool mirror = false, bool transparent = false, double n = 1.0) 
    : C(C), R(R), Geometry(rho, mirror, transparent, n) {};

    Vector C;
    double R;

    Intersection intersect(const Ray& r, bool& is_in) const override {
        Intersection I(rho, n, mirror);

        double delta = sqr(dot(r.u, r.O - C)) - ((r.O - C).norm2() - sqr(R));  // the discriminant

        // if delta < 0, there is no intersection
        if (delta < 0) {
            I.intersects = false;
            return I;
        }

        double x = dot(r.u, C - r.O);
        double t1 = x - sqrt(delta);
        double t2 = x + sqrt(delta);
        double t;
        
        if (t2 < 0) {
            I.intersects = false;
            return I;
        }  

        if (t1 > 0) {
            I.t = t1;
        }
        else {
            I.t = t2;
        }
     
        I.P = r.O + I.t * r.u;

        I.N = (I.P - C) / (I.P - C).norm();

        I.intersects = true;
        return I;
    }
};

class Scene {
    public:
        std::vector<Geometry*> shapes;

        void add(Geometry* p) {
            shapes.push_back(p);
        }

        Intersection intersect(const Ray& r, bool& is_in) { // t is the point we are looking for
            Intersection tempI;
            Intersection I;
            double closest_t = std::numeric_limits<double>::max();

            for (size_t i = 0; i < shapes.size(); ++i) {
                tempI = shapes[i]->intersect(r, is_in);
                if (tempI.intersects) {
                    if (tempI.t < closest_t) {
                        closest_t = tempI.t;
                        I = tempI;
                        I.id = i;
                    }
                } 
            }

            return I;
        }

        Vector getColour(const Ray& ray, int depth, Vector lpos, double I, bool& is_in) {
            // Base Case
            if (depth <= 0) {
                return Vector(0, 0, 0);
            }

            Intersection In = intersect(ray, is_in);
            Vector colour = Vector(0, 0, 0);

            double eps = 1e-5;

            if (In.intersects) {
                Geometry* S = shapes[In.id];
                is_in = dot(ray.u, In.N) < 0 ? false : true;

                if (S->mirror) {
                    Vector refl_dir = ray.u - In.N * (2 * dot(ray.u, In.N));
                    refl_dir.normalize();
                    return getColour(Ray(In.P + In.N * eps, refl_dir), depth - 1, lpos, I, is_in);
                }

                else if (S->transparent) {
                    // incident ray
                    Vector i = ray.u; 
                    i.normalize();
                    double Ni = dot(In.N, i);

                    // fresnel reflection -- Bruno Iorio helped me with the logic here :)

                    double n1 = 1.0; // refarction index for air
                    double n2 = S->n; // refraction index for sphere
                    bool entering = Ni < 0;

                    if (!entering) {
                        std::swap(n1, n2);
                        In.N = -1 * In.N;
                        Ni = dot(In.N, i);
                    }

                    Vector refl_origin = In.P + In.N * eps;

                    double ratio = n1 / n2;
                    double c = 1 - sqr(ratio) * (1 - sqr(Ni));

                    if (c < 0) { 
                        // total internal reflection
                        Vector refl_dir = i - 2 * Ni * In.N;
                        refl_dir.normalize();
                        Ray refl = Ray(refl_origin, refl_dir);
                        return getColour(refl, depth - 1, lpos, I, is_in);
                    }
                    else {
                        // refraction -- apply Fresnel
                        Vector vt = ratio * (i - Ni * In.N);
                        Vector vn = -1 * In.N * sqrt(c);
                        Vector vf = vt + vn;
                        vf.normalize();
                        Vector refr_origin = In.P - In.N * eps;

                        double k0 = sqr(n1 - n2) / sqr(n1 + n2);
                        double R = k0 + (1 - k0) * pow((1 - std::abs(Ni)), 5); 
                        double T = 1 - R;

                        Ray refr = Ray(refr_origin, vf);
                        Vector refr_colour = getColour(refr, depth - 1, lpos, I, is_in);

                        Vector refl_dir = i - 2 * Ni * In.N;
                        refl_dir.normalize();
                        Ray refl = Ray(refl_origin, refl_dir);
                        Vector refl_colour = getColour(refl, depth - 1, lpos, I, is_in);
                        
                        return refl_colour * R + refr_colour * T;

                    }
                }
                
                Vector random_dir = random_cos(In.N);
                Ray random_ray = Ray(In.P, random_dir);
                Vector colourindirect(In.rho * getColour(random_ray, depth - 1, lpos, I, is_in));
                colour = colour + colourindirect;
                

                // check for shadow
                if (isShadow(In.P, In.N, lpos, is_in)) {
                    return colour;
                }

                else {
                    // direct lighting

                    Vector L = lpos - In.P;
                    double dist = L.norm2();
                    Vector material = In.rho / M_PI;
                    double const1 = I / (4 * M_PI * dist);
                    double pos_dot = std::max(0.0, dot(In.N, L / L.norm()));
                    Vector colourdirect = material * const1 * pos_dot;

                    colour = colour + colourdirect;
                    return colour;    
                    
                }
            }

            return colour;
        }

        bool isShadow(const Vector& P, const Vector& N, const Vector& lpos, bool& is_in) {
            Vector ldir = lpos - P;
            double ldist = ldir.norm();
            ldir.normalize();

            Ray shadowRay = Ray(P + N * 1e-4, ldir);
            
            Intersection tempI;
            tempI = intersect(shadowRay, is_in);

            if (tempI.intersects) {
                double hdist = (tempI.P - P).norm();
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
    int max_depth = 5;
    int rpp = 50; // rays per pixel
    Scene scene;

    // Subjects of the Image
    // normal 3 spheres
    /*
    scene.add(new Sphere(Vector(-20, -1, 0), 9, Vector(0.7, 0.1, 0.5), false, true, 1.5));
    scene.add(new Sphere(Vector(0, -1, 0), 9, Vector(0.7, 0.1, 0.5), true));
    scene.add(new Sphere(Vector(20, -1, 0), 9, Vector(0.7, 0.1, 0.5)));
    */

    // 3 spheres, different positions
    scene.add(new Sphere(Vector(-12, 0, 15), 10, Vector(0.7, 0.1, 0.5), false, true, 1.5));
    scene.add(new Sphere(Vector(0, 0, 5), 10, Vector(0.9, 0.7, 0.9)));
    scene.add(new Sphere(Vector(12, 0, -10), 10, Vector(0.7, 0.1, 0.5), true));


    // cat    
    /*
    TriangleMesh bausch(0.6, Vector(0, -10, 0), Vector(0.8, 0.1, 0.9));
    bausch.readOBJ("cat.obj");
    bausch.createBVH();
    scene.add(&bausch);
    // std::cout << "Bausch added" << std::endl;
    */
    
    // Background
    scene.add(new Sphere(Vector(0,0,1000), 940, Vector(0.9,0.2,0.9))); // left-wall
    scene.add(new Sphere(Vector(0,0,-1000), 940, Vector(0.6,0.5,0.1))); // right-wall
    scene.add(new Sphere(Vector(0,-1000,0), 990, Vector(0.3,0.4,0.7))); // floor
    scene.add(new Sphere(Vector(0,1000,0), 940, Vector(0.2,0.5,0.9))); // ceiling
    scene.add(new Sphere(Vector(-1000,0,0), 940, Vector(0.4,0.8,0.7))); // back-wall
    scene.add(new Sphere(Vector(1000,0,0), 940, Vector(0.9,0.4,0.3))); // other wall

    // Scene set-up
    std::vector<unsigned char> image(W * H * 3, 0);
    Vector camerao = Vector(0, 0, 55);
    double fieldofview = 60 * M_PI / 180;
    double aperture = 1.0;               // strength of the blur
    double focal_dist = 55.0;            // distance from camera to focus point 

    // Light settings
    double I = 0.7E5;
    Vector lpos = Vector(-10, 20, 40);
    
    bool flag = false;

    #pragma omp parallel for schedule(dynamic, 1) // parallelises code
    for (int i = 0; i < H; i++) {

        for (int j = 0; j < W; j++) {
            // std::cout << "Height:" << i << ", Width:"<< j<< "\n";
            Vector colour = Vector(0, 0, 0);
            double x, y;

            for (int k = 0; k < rpp; ++k) {
                boxMuller(0.5, x, y); // alter stdev here
                // standard rendering
                /*
                Vector pixel = Vector(camerao[0] + (j+x) + 0.5 - W/2,
                                      camerao[1] - (i+y) - 0.5 + H/2,
                                      camerao[2] - W/(2 * tan(fieldofview/2))
                                );
                Vector ray_dir = pixel - camerao;
                ray_dir.normalize();
                Ray ray = Ray(camerao, ray_dir);
                colour = colour + scene.getColour(ray, max_depth, lpos, I, flag);
                */
               Vector pixel = Vector(j + x + 0.5 - W / 2,
                                     H / 2 - (i + y + 0.5),
                                    -W / (2 * tan(fieldofview / 2)));  

                pixel.normalize();
                Vector focal_point = camerao + pixel * focal_dist;

                double lens_x, lens_y;
                boxMuller(aperture, lens_x, lens_y);
                Vector lens_offset = Vector(lens_x, lens_y, 0);

                Vector ray_origin = camerao + lens_offset;
                Vector ray_dir = (focal_point - ray_origin);
                ray_dir.normalize();
                Ray ray(ray_origin, ray_dir);

                colour = colour + scene.getColour(ray, max_depth, lpos, I, flag);
            }

            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(colour[0] / rpp, 1 / 2.2) * 255);
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(colour[1] / rpp, 1 / 2.2) * 255);
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(colour[2] / rpp, 1 / 2.2) * 255);
        }
    }
    std::cout << "Finished rendering\n"; //debug
    stbi_write_png("cat7.png", W, H, 3, &image[0], 0);
    std::cout << "Finished pic\n"; //debug

    auto stop = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
    std::cout << "Time: " << time.count() << " milliseconds." << std::endl;
 
    return 0;
}
