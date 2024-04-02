#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>

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
};

class Sphere {
public:
    Vector origin;
    double radius;
    Vector albedo;
    bool mirror;
    explicit Sphere(Vector o, double R, Vector c){
        origin = o;
        radius = R;
        albedo = c;
        if (c.data[0] == -1){
            mirror = true;
        }
        else{
            mirror = false;
        }
    }
    Intersection intersect(Ray &r){
        Vector omc = r.origin - origin;
        double delta = pow(dot(r.unit, omc), 2) - (dot(omc, omc) - pow(radius, 2));
        if (delta<0){
            return {false, r.origin, 0};
        }
        double sq_delta = sqrt(delta);
        double t = dot(r.unit, origin-r.origin) - sq_delta;
        if (t<0){
            t += 2*sq_delta;
            if (t<0){
                return {false, r.origin, 0};
            }
        }
        return {true, r.origin + r.unit*t, t};
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

Vector get_color(std::vector<Sphere> Scene, std::vector<Light> Lights, Ray pr, int mirror_depth = 10){
    Vector color;
    Cast cast = scene_intersect(Scene, pr);
    if (cast.intersect.flag == true){
        Vector sphere_normal = cast.intersect.position - (*cast.sphere).origin;
        sphere_normal.normalize();
        Vector epsilon_above = cast.intersect.position + sphere_normal/100000;
        if (cast.sphere->mirror & mirror_depth>0){
            return get_color(Scene, Lights, Ray(epsilon_above, pr.unit - 2 * dot(pr.unit, sphere_normal) * sphere_normal), mirror_depth-1);
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
                color = color + ((Lights[k].intensity/(4*PI*(to_light).norm2())) * (albedo/PI) * std::max((double)0, dot(sphere_normal, to_light/to_light.norm())));
            }
        }
    }
    return color;
}

int main(){
    Vector mirror_vect = Vector(-1, -1, -1);
    std::vector<Sphere> Scene{  Sphere(Vector(0,0,0), 10, Vector(170, 10, 170)),        // center ball
                                Sphere(Vector(0, 1000, 0), 940, Vector(255, 0, 0)),     // top red
                                Sphere(Vector(0, 0, -1000), 940, Vector(0, 255, 0)),    // end green
                                Sphere(Vector(0, -1000, 0), 990, Vector(0, 0, 255)),    // bottom blue
                                Sphere(Vector(0, 0, 1000), 940, Vector(132, 46, 27)),   // back brown
                                Sphere(Vector(15, 5, -5), 3, Vector(255, 255, 0)),      // small yellow
                                Sphere(Vector(-15, 5, -5), 4, mirror_vect)              // left mirror
                                };
    std::vector<Light> Lights{{Vector(-10, 20, 40), 7*10000000}, {Vector(15, 0, -5), 6*1000000}};
    

    place_camera_scene(Scene, Lights, Vector(0, 0, 55));

    int W = 512;
    int H = 512;
 
    std::vector<unsigned char> image(W * H * 3, 0);

    for (int i = 0; i < H; i++) {
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
    

    stbi_write_png("image.png", W, H, 3, &image[0], 0);
 
    return 0;
}