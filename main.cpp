#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include <thread>
#include <random>

#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821

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
Vector operator-(const Vector& a){
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
double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}
 
class Ray {
public:
    Vector origin;
    Vector unit;
    explicit Ray(Vector o, Vector u){
        origin = o;
        unit = u;
        unit.normalize();
    }
};

struct Intersection {
    bool flag;
    Vector position;
    double t;
    bool inside;
};

Intersection make_intersection(bool flag, Vector position, double t, bool inside){
    return {flag, position, t, inside};
}

class Sphere {
public:
    Vector origin;
    double radius;
    Vector albedo;
    bool mirror;
    bool transp;
    double refraction;
    explicit Sphere(Vector o, double R, Vector c, double refr = -1){
        origin = o;
        radius = R;
        albedo = c;
        refraction = refr;
        if (refraction == -1){
            mirror = false;
            transp = false;
        }
        else if (refraction == 0){
            transp = false;
            mirror = true;
        }
        else{
            transp = true;
            mirror = false;
        }
    }
    Intersection intersect(Ray &r){
        Vector omc = r.origin - origin;
        double delta = pow(dot(r.unit, omc), 2) - (dot(omc, omc) - pow(radius, 2));
        if (delta<0){
            return make_intersection(false, r.origin, 0, false);
        }
        double sq_delta = sqrt(delta);
        double t = dot(r.unit, origin-r.origin) - sq_delta;
        bool inside = false;
        if (t<0){
            t += 2*sq_delta;
            if (t<0){
                return make_intersection(false, r.origin, 0, false);
            }
            inside = true;
        }
        return make_intersection(true, r.origin + r.unit*t, t, inside);
    }
};

struct Cast{
    Intersection intersect;
    Sphere* sphere;
};

Cast scene_intersect(std::vector<Sphere> scene, Ray &r){
    if (scene.size() == 0){
        return {false, r.origin, 0};
    }
    Cast best = {scene[0].intersect(r), &(scene[0])};
    Intersection current_intersect;
    for (size_t i=1; i<scene.size(); i++){
        current_intersect = scene[i].intersect(r);
        if (best.intersect.flag == false || (current_intersect.flag == true && current_intersect.t < best.intersect.t)){
            best = {current_intersect, &(scene[i])};
        }
    }
    return best;
}

Ray pixel_ray(int W, int H, int i, int j){
    long double alpha = PI * 60/180;
    Vector u = Vector(j-(W/2)+0.5, (H/2)-i-0.5, -W/(2*tan(alpha/2)));
    return Ray(Vector(0, 0, 0), u);
}

struct Light{
    Vector position;
    int intensity;
};

void place_camera_scene(std::vector<Sphere> &scene, std::vector<Light> &lights, Vector camera_pos){
    for (size_t i=0; i<scene.size(); i++){
        scene[i].origin = scene[i].origin - camera_pos;
    }
    for (size_t i=0; i<lights.size(); i++){
        lights[i].position = lights[i].position - camera_pos;
    }
}

void gamma_correction(Vector& color){
    color[0] = std::min((double)255, std::max((double)0, pow(color[0], 1/2.2)));
    color[1] = std::min((double)255, std::max((double)0, pow(color[1], 1/2.2)));
    color[2] = std::min((double)255, std::max((double)0, pow(color[2], 1/2.2)));
}

Vector get_color(std::vector<Sphere> Scene, std::vector<Light> Lights, Ray pr, int reflections_depth = 20){
    Vector color;
    Cast cast = scene_intersect(Scene, pr);
    double epsilon = 1.0/100000;
    if (cast.intersect.flag == true){
        Vector sphere_normal = cast.intersect.position - (*cast.sphere).origin;
        sphere_normal.normalize();
        Vector normal_towards_ray = sphere_normal;
        if (cast.intersect.inside == true){
            normal_towards_ray = -sphere_normal;
        }
        double dotwin = dot(pr.unit, normal_towards_ray);
        Vector epsilon_above = cast.intersect.position + normal_towards_ray * epsilon;
        if (cast.sphere->mirror & reflections_depth>0){
            return get_color(Scene, Lights, Ray(epsilon_above, pr.unit - 2 * dotwin * normal_towards_ray), reflections_depth-1);
        }
        else if (cast.sphere->transp){
            // We always assume the sphere is standing in air
            Vector epsilon_after = cast.intersect.position - normal_towards_ray * epsilon;
            double n1n2 = 1.0/cast.sphere->refraction;
            if (cast.intersect.inside == true){
                n1n2 = cast.sphere->refraction;
            }
            Vector tangential_dir = n1n2 * (pr.unit - dotwin * normal_towards_ray);
            double in_sqrt = 1 - (pow(n1n2,2) * (1 - pow(dotwin,2)));
            if (in_sqrt<0){
                std::cout << "WARNING: issue in refraction handling; transparent surface with mirror behaviour from value of refraction index" << std::endl;
                cast.sphere->mirror = true;
                cast.sphere->refraction = 0;
                cast.sphere->transp = false;
                return get_color(Scene, Lights, pr, reflections_depth);
            }
            Vector normal_dir = - normal_towards_ray * sqrt(in_sqrt);
            Vector refracted_direction = tangential_dir + normal_dir;
            Ray reflected_ray = Ray(epsilon_after, refracted_direction); // TODO change
            return get_color(Scene, Lights, reflected_ray, reflections_depth-1);
        }
        Vector albedo = (*cast.sphere).albedo;

        for (int k=0; k<Lights.size(); k++){
            // First test if there is a shadow
            Vector to_shadow = Lights[k].position - epsilon_above;
            Ray shadow_ray = Ray(epsilon_above, to_shadow);
            Cast shadow_intersection = scene_intersect(Scene, shadow_ray);
            if (!shadow_intersection.intersect.flag | to_shadow.norm2() < (epsilon_above - shadow_intersection.intersect.position).norm2()){
                // Then add the light to the pixel
                Vector to_light = Lights[k].position - cast.intersect.position;
                color = color + ((Lights[k].intensity/(4*PI*(to_light).norm2())) * (albedo/PI) * std::max((double)0, dot(normal_towards_ray, to_light/to_light.norm())));
            }
        }
    }
    return color;
}


void random_cos(const Vector &N){
    std::default_random_engine gen;
    std::uniform_real_distribution<double> udis(0.0,1.0);
    double r1 = udis(gen);
    double r2 = udis(gen);
    double x = cos(2*PI*r1) * sqrt(1 - r2);
    double y = sin(2*PI*r1) * sqrt(1 - r2);
    double z = sqrt(r2);

    double minabs_N = std::min(std::min(abs(N.data[0]), abs(N.data[1])), abs(N.data[2]));
    Vector T1;
    if (abs(N.data[0]) <= abs(N.data[1]) && abs(N.data[0]) <= abs(N.data[2])){
        T1 = Vector(0, -N.data[1], N.data[2]);
    }
    else if (abs(N.data[1]) <= abs(N.data[2])){
        T1 = Vector(-N.data[2], 0, N.data[0]);
    }
    else{
        T1 = Vector(-N.data[1], N.data[0], 0);
    }
    T1.normalize();
    Vector T2 = cross(N, T1);

    Vector V = x*T1 + y*T2 + z*N;
}


void concurrent_line(std::vector<Sphere> Scene, std::vector<Light> Lights, int W, int H, int i0, size_t block_size, std::vector<unsigned char> &image){
    for (int i = i0; i < i0+block_size; i++){
        for (int j = 0; j < W; j++) {
            Ray pr = pixel_ray(W, H, i, j);
            Vector color = Vector(0,0,0);
            color = get_color(Scene, Lights, pr);
                
            //std::cout << color[0] << " " << color[1] << " " << color[2] << std::endl;
            gamma_correction(color);
            image[(i * W + j) * 3 + 0] = color.data[0];
            image[(i * W + j) * 3 + 1] = color.data[1];
            image[(i * W + j) * 3 + 2] = color.data[2];
        }
    }
}

int main(){
    sampling_todo();
    return 0;
    Vector empty_vec = Vector(-1, -1, -1);
    std::vector<Sphere> Scene{  Sphere(Vector(0,0,0), 10, Vector(170, 10, 170)),        // center ball
                                Sphere(Vector(0, 1000, 0), 940, Vector(255, 0, 0)),     // top red
                                Sphere(Vector(0, 0, -1000), 940, Vector(0, 255, 0)),    // end green
                                Sphere(Vector(0, -1000, 0), 990, Vector(0, 0, 255)),    // bottom blue
                                Sphere(Vector(0, 0, 1000), 940, Vector(132, 46, 27)),   // back brown
                                Sphere(Vector(1000, 0, 0), 940, Vector(255, 0, 255)),   // right pink
                                Sphere(Vector(-1000, 0, 0), 940, Vector(255, 255, 0)),  // left orange
                                Sphere(Vector(15, 5, -5), 3, Vector(255, 255, 0)),      // small yellow
                                Sphere(Vector(-20, 21, -13), 10, empty_vec, 0),         // left mirror
                                Sphere(Vector(-10, 0, 15), 8, empty_vec, 1.49)          // left lens
                                };
    std::vector<Light> Lights{{Vector(-10, 20, 40), 7*10000000}, {Vector(15, 0, -5), 6*1000000}};
    

    place_camera_scene(Scene, Lights, Vector(0, 0, 55));

    int W = 512;
    int H = 512;
 
    std::vector<unsigned char> image(W * H * 3, 0);
    size_t n_threads = 48;
    size_t block_size = H / (n_threads-1);
    std::vector<std::thread> threads(n_threads);
    
    for (int i = 0; i < n_threads-1; i++) {
        threads[i] = std::thread(&concurrent_line, Scene, Lights, W, H, i*block_size, block_size, std::ref(image));
    }

    for (int i = (n_threads-1)*block_size; i < W; i++){
        for (int j = 0; j < W; j++) {
            Ray pr = pixel_ray(W, H, i, j);
            Vector color = Vector(0,0,0);
            color = get_color(Scene, Lights, pr);

            gamma_correction(color);
            image[(i * W + j) * 3 + 0] = color.data[0];
            image[(i * W + j) * 3 + 1] = color.data[1];
            image[(i * W + j) * 3 + 2] = color.data[2];
        }
    }

    for (int i = 0; i < n_threads-1; i++){
        threads[i].join();
    }

    stbi_write_png("image.png", W, H, 3, &image[0], 0);
 
    return 0;
}