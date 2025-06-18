#include <vector>
#include <string>
#include <cstdio>  
#include <cmath>
#include <random>
#include <chrono>
#include <iostream>
#include <sstream>
#include <stdio.h>

#include "lbfgs.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define VOL_FLUID 0.6
#define M_PI 3.14159265359

static std::default_random_engine engine(10);  
static std::uniform_real_distribution<double> uniform(0, 1);

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

Vector operator*( const Vector& a, const Vector& b ) {
    return Vector( a[0] * b[0], a[1] * b[1], a[2] * b[2] );
}

class Polygon {  
public:
    std::vector<Vector> vertices;

    double integral_square_distance(const Vector& Pi ){
        if (vertices.size() < 3) return 0;

        double s = 0;

        for (int i = 1; i < vertices.size() - 1; i++) {
            Vector c[3] = { vertices[0], vertices[i], vertices[i + 1]};

            double integral = 0;

            // formula from the slides
            for (int k = 0; k < 3; k++ ) {
                for (int l = k; l < 3; l++ ) {
                    integral += dot(c[k] - Pi, c[l] - Pi);
                }
            }

            // cross product
            Vector edge1 = c[1] - c[0];
            Vector edge2 = c[2] - c[0];
            double areaT = 0.5 * std::abs(edge1[0] * edge2[1] - edge1[1] * edge2[0]);
            s += integral * areaT / 6;
        }

        return s;
    }

    double area() {
        if(vertices.size() < 3) return 0;

        double s = 0;

        for (int i = 0; i < vertices.size(); i++) {
            int ip = (i == vertices.size() - 1) ? 0 : (i + 1);
            s += vertices[i][0] * vertices[ip][1] - vertices[ip][0] * vertices[i][1];
        }

        return std::abs(s) / 2;
    }

    Vector centroid() {
        if (vertices.size() < 3) return Vector (0, 0, 0);

        int N = vertices.size();
        Vector c = Vector(0, 0, 0);

        for( int i = 0; i < N; i++ ) {
            int ip = (i == N - 1) ? 0 : (i + 1);  
            double crossP = vertices[i][0] * vertices[ip][1] - vertices[ip][0] * vertices[i][1];
            c[0] = c[0] + (vertices[i][0] + vertices[ip][0]) * crossP;  
            c[1] = c[1] + (vertices[i][1] + vertices[ip][1]) * crossP;
        }

        // Andreea Patarlageanu helped me with this formula here
        double A = area();
        c[0]= -c[0] / (6 * A);
        c[1] = -c[1] / (6 * A);
        return c;
    }
};	

void save_svg(const std::vector<Polygon>& polygons, std::string filename, const std::vector<Vector> *points = NULL, std::string fillcol = "none") {
        FILE* f = fopen(filename.c_str(), "w+");
        fprintf(f, "<svg xmlns = \"http://www.w3.org/2000/svg\" width = \"1000\" height = \"1000\">\n");
        for (int i = 0; i < polygons.size(); i++) {
            fprintf(f, "<g>\n");
            fprintf(f, "<polygon points = \"");
            for (int j = 0; j < polygons[i].vertices.size(); j++) {
                fprintf(f, "%3.3f, %3.3f ", (polygons[i].vertices[j][0] * 1000), (1000 - polygons[i].vertices[j][1] * 1000));
            }
            fprintf(f, "\"\nfill = \"%s\" stroke = \"black\"/>\n", fillcol.c_str());
            fprintf(f, "</g>\n");
        }
    
        if (points) {
            fprintf(f, "<g>\n");		
            for (int i = 0; i < points->size(); i++) {
                fprintf(f, "<circle cx = \"%3.3f\" cy = \"%3.3f\" r = \"3\" />\n", (*points)[i][0]*1000., 1000.-(*points)[i][1]*1000);
            }
            fprintf(f, "</g>\n");
    
        }
    
        fprintf(f, "</svg>\n");
        fclose(f);
}

// from class
int sgn(double x) {
    if(x > 0) return -1;
    if(x < 0) return 1;
    return 0;
}

void save_frame(const std::vector<Polygon> &cells, std::string filename, int frameid = 0) {
        int W = 500, H = 500 ;
		std::vector<unsigned char> image(W*H * 3, 255);

        #pragma omp parallel for schedule(dynamic)
		for (int i = 0; i < cells.size(); i++) {

			double bminx = 1E9, bminy = 1E9, bmaxx = -1E9, bmaxy = -1E9;
			for (int j = 0; j < cells[i].vertices.size(); j++) {
				bminx = std::min(bminx, cells[i].vertices[j][0]);
				bminy = std::min(bminy, cells[i].vertices[j][1]);
				bmaxx = std::max(bmaxx, cells[i].vertices[j][0]);
				bmaxy = std::max(bmaxy, cells[i].vertices[j][1]);
			}
			bminx = std::min(W-1., std::max(0., W * bminx));
			bminy = std::min(H-1., std::max(0., H * bminy));
			bmaxx = std::max(W-1., std::max(0., W * bmaxx));
			bmaxy = std::max(H-1., std::max(0., H * bmaxy));

			for (int y = bminy; y < bmaxy; y++) {
				for (int x = bminx; x < bmaxx; x++) {
					int prevSign = 0;
					bool isInside = true;
					double mindistEdge = 1E9;
					for (int j = 0; j < cells[i].vertices.size(); j++) {
						double x0 = cells[i].vertices[j][0] * W;
						double y0 = cells[i].vertices[j][1] * H;
						double x1 = cells[i].vertices[(j + 1) % cells[i].vertices.size()][0] * W;
						double y1 = cells[i].vertices[(j + 1) % cells[i].vertices.size()][1] * H;
						double det = (x - x0)*(y1-y0) - (y - y0)*(x1-x0);
						int sign = sgn(det);
						if (prevSign == 0) prevSign = sign; else
							if (sign == 0) sign = prevSign; else
							if (sign != prevSign) {
								isInside = false;
								break;
							}
						prevSign = sign;
						double edgeLen = sqrt((x1 - x0)*(x1 - x0) + (y1 - y0)*(y1 - y0));
						double distEdge = std::abs(det)/ edgeLen;
						double dotp = (x - x0)*(x1 - x0) + (y - y0)*(y1 - y0);
						if (dotp<0 || dotp>edgeLen*edgeLen) distEdge = 1E9;
						mindistEdge = std::min(mindistEdge, distEdge);
					}
					if (isInside) { // to add color
						if (i < 100) {   // the N first particles may represent fluid, displayed in blue
							image[((H - y - 1)*W + x) * 3] = 0;
							image[((H - y - 1)*W + x) * 3 + 1] = 0;
							image[((H - y - 1)*W + x) * 3 + 2] = 255;
						}
						if (mindistEdge <= 2) {
							image[((H - y - 1)*W + x) * 3] = 0;
							image[((H - y - 1)*W + x) * 3 + 1] = 0;
							image[((H - y - 1)*W + x) * 3 + 2] = 0;
						}

					}
					
				}
			}
		}
		std::ostringstream os;
		os << filename << frameid << ".png";
		stbi_write_png(os.str().c_str(), W, H, 3, &image[0], 0);
	}


class VoronoiDiagram{
public:
	VoronoiDiagram() {
        N_disk = 100;
        unit_disk.resize( N_disk );
        for( int i = 0; i < N_disk; i++ ) {
            double theta = -i * 2 * M_PI / (double) N_disk;
            unit_disk[i] = Vector(sin(theta), cos(theta), 0);
        }
    };

    Polygon clip_by_bisector( const Polygon& V, const Vector&P0, const Vector& Pi, const double w0, const double wi ){
		Polygon out;
		Vector M = (P0 + Pi) * 0.5;
        double sqrdist = (P0 - Pi).norm2();
        Vector Mprime = M + (((w0 - wi) / (2 * sqrdist)) * (Pi - P0));
        
		for(int i = 0; i < V.vertices.size(); i++ ){
            int ip = (i == 0) ? V.vertices.size() -1 : i - 1;
			const Vector &A = V.vertices[ip];
			const Vector &B = V.vertices[i];

			if((B - P0).norm2() - w0 <= (B - Pi).norm2() - wi) { 
				if(( A - P0).norm2() - w0 >= (A - Pi).norm2() - wi) {
					double t = dot(Mprime - A, Pi - P0) / dot(B - A, Pi - P0);
					Vector P = A + t * (B - A );
					out.vertices.push_back(P);
				}

				out.vertices.push_back(B);
			}

			else {
				if((A -P0).norm2() - w0 <= (A - Pi).norm2() - wi) {
					double t = dot(Mprime - A, Pi - P0) / dot(B - A, Pi - P0);
					Vector P = A + t * (B - A);
					out.vertices.push_back(P);
				}
			}
		}
        
		return out;
	}

    Polygon clip_by_edge( const Polygon& V, const Vector& u, const Vector& v ) {
        // Suthrman-Hidgam algorithm
        const Vector N(v[1] - u[1], u[0] - v[0], 0);
        Polygon result;
        result.vertices.reserve( V.vertices.size() + 1);

        for (int i = 0 ; i < V.vertices.size() ; i++) {
            int ip = (i == 0) ? V.vertices.size() - 1 : i - 1;
            const Vector& A = V.vertices[ip];
            const Vector& B = V.vertices[i];

            if (dot(u - B, N) >= 0) {
                if (dot(u - A, N) < 0) {
                    double t = dot(u - A, N) / dot(B - A, N);
                    Vector P = A + t * (B - A);
                    result.vertices.push_back(P);
                }
                result.vertices.push_back(B);
            } 

            else if (dot( u - A, N ) >= 0) {
                double t = dot(u - A, N) / dot(B - A, N);
                Vector P = A + t * (B - A);
                result.vertices.push_back(P);
            }
        }

        return result;
    }

	void compute() {
		Polygon square;
		square.vertices.resize(4);
        square.vertices[0] = Vector(0, 0, 0);
        square.vertices[1] = Vector(0, 1, 0);
        square.vertices[2] = Vector(1, 1, 0);
        square.vertices[3] = Vector(1, 0, 0);

		cells.resize(points.size());

        #pragma omp parallel for schedule(dynamic, 1)
		for (int i = 0; i < points.size(); i++) {
			Polygon V = square;

			for (int j = 0; j < points.size(); j++) {
				if(i == j) continue;

				V = clip_by_bisector(V, points[i], points[j], weights[i], weights[j]);
			}

            double delta = weights[i] - weights[weights.size() - 1];
            double r = sqrt(delta);

            for (int k = 0; k < N_disk; k++) {
                Vector u = unit_disk[k] * r + points[i];
                Vector v = unit_disk[(k + 1) % N_disk] * r + points[i];
                V = clip_by_edge(V, u, v);
            }

			cells[i] = V;
		}
	}

	std::vector<Vector> points;
    std::vector<double> weights;
	std::vector<Polygon> cells; // diagram in lecture

    std::vector<Vector> unit_disk;
    int N_disk;

};

class OptimalTransport{
public:
    OptimalTransport(){};
    VoronoiDiagram vor;
    // std::vector<double> target_areas;

    void optimise();
    
    // Andreea Patarlageanu gave me the idea to integrate evaluate and progress in the OT class.

    static lbfgsfloatval_t _evaluate (
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
    )
    { return reinterpret_cast<OptimalTransport*>(instance)->evaluate(x, g, n, step);}

    lbfgsfloatval_t evaluate(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
    )
    {
        int np = n - 1;
        memcpy(&(vor.weights[0]), x, n * sizeof(x[0]));
        vor.compute();

        lbfgsfloatval_t fx = 0.0;
        double sum_fluid_areas = 0;

        // formula from class
        for (int i = 0; i < n - 1; i++) {
            double current_area = vor.cells[i].area();
            sum_fluid_areas += current_area;
            g[i] = -(VOL_FLUID / np - current_area);
            fx += vor.cells[i].integral_square_distance(vor.points[i]) - x[i] * (current_area - VOL_FLUID / np);
        }

        double estimated_air_volume = 1 - sum_fluid_areas;
        double desired_air_volume = 1.0 - VOL_FLUID;
        g[n - 1] = -(desired_air_volume - estimated_air_volume);
        fx = fx + x[n - 1] * (desired_air_volume - estimated_air_volume);

        return -fx;
    }

    static int _progress(
        void *instance,
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    { return reinterpret_cast<OptimalTransport*>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);}

    int progress(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
    )
    {
        //printf("Iteration %d:\n", k);
        //printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
        //printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
        //printf("\n");
        return 0;
    }
};

void OptimalTransport::optimise(){
        int N = vor.weights.size();
        lbfgsfloatval_t fx;
        std::vector<double> weights(N, 0);
        memcpy(&weights[0], &vor.weights[0], N * sizeof(weights[0]));

        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);

        int ret = lbfgs(N, &weights[0], &fx, _evaluate, _progress, (void*)this, &param);
        
        memcpy(&vor.weights[0], &weights[0], N * sizeof(weights[0]));
    }


// Adreea Patarlageanu helped me with this class
// Specifically, the initilisation and time_step
class Fluid{
public:
    Fluid(int N = 1000, int m = 200) : N(N), m(m), dt(0.002) {
        particles.resize(N);
        velocities.resize(N, Vector(0, 0, 0));

        for (int i = 0; i < N; i++) {
            particles[i] = Vector(uniform(engine), uniform(engine), 0);
        }
        
        ot.vor.points = particles;
        ot.vor.weights.resize(N + 1);
        for (auto& w : ot.vor.weights) {
            w = 1;
        }
        ot.vor.weights[N] = 0.99;
    };

    void time_step(int frames){
        double eps = 0.004; // spring stiffness
        Vector g = Vector(0, -9.81, 0); // gravity
        ot.vor.points = particles;
        ot.optimise();

        for (int i = 0; i < frames; i++) {
            ot.vor.points = particles;
            ot.optimise();
            for (int j = 0; j < N; j++) {
                Vector center = ot.vor.cells[j].centroid();
                Vector sumforces = (center - particles[j]) / (eps * eps) + m * g;
                velocities[j] = velocities[j] + dt / m * (center - particles[j]) / (eps * eps);
                particles[j] = particles[j] + dt * velocities[j];
            }

            save_frame( ot.vor.cells, "./fluidpics/test_fluid", i);
        }
    }

    OptimalTransport ot;
    std::vector<Vector> particles;
    std::vector<Vector> velocities;
    double dt;
    int N;
    int m;
};

int main() {
	int N = 85;  //particles
    int m = 150; // mass
    double frames = 100; // number of frames

    std::cout << "Program started!";

    Fluid fluid(N, m);    

    auto s = std::chrono::high_resolution_clock::now();

    fluid.time_step(frames);

    auto e = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> time = e - s;

    std::cout << "Fluid compute time: " << time.count() << " seconds for " << frames <<  "frames" << std::endl;

    exit(0);

    // code after exit does not run

	VoronoiDiagram Vor;

	for( int i = 0; i < N; i++ ) {
		Vor.points.push_back(Vector(uniform(engine), uniform(engine), 0));
        	Vor.weights.push_back(0);
	}

    std::cout << "Init done!";

    auto start = std::chrono::high_resolution_clock::now();

    Vor.compute();

    std::cout << "Compute done!";

    OptimalTransport ot;
    ot.vor = Vor;
    ot.optimise();

    std::cout << "Optimise done!";

    auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;

	std::cout << "Voronoi compute time: " << elapsed.count() << " seconds" << std::endl;

	save_svg(ot.vor.cells, "test.svg", &ot.vor.points);
}
