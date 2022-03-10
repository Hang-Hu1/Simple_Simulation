#ifndef SCENE_SCENE_H
#define SCENE_SCENE_H

#include "Eigen/SparseLU"
#include "Eigen/Core"
#include <vector>

struct HelloWorld {
    void greet();
};

class Vec3 {
public:
    Vec3();
    Vec3(float, float, float);

    float value(int) const;

    Vec3 plus(const Vec3&) const;
    Vec3 minus(const Vec3&) const;
    Vec3 multiply(float) const;
    Vec3 divide(float) const;

private:
    float element0;
    float element1;
    float element2;
};


class FreeFaller {
public:
    FreeFaller(float, const Vec3&, const Vec3&);

    void step(float, const Vec3&);
    Vec3 position() const;
    Vec3 velocity() const;

private:
    float m;
    Eigen::Vector3d x;
    Eigen::Vector3d v;
    Eigen::Vector3d g;
};




#endif