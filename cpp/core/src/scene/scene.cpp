#include "scene/scene.h"

void HelloWorld::greet() {
    printf("Hello world!\n");
}


Vec3::Vec3() { element0 = 0.0; element1 = 0.0; element2 = 0.0;}
Vec3::Vec3(float a, float b, float c) { element0 = a; element1 = b; element2 = c;}

float Vec3::value(int index) const {
    if (index == 0) return element0;
    else if (index == 1) return element1;
    else return element2;
}

Vec3 Vec3::plus(const Vec3 &v) const {
    return Vec3(element0 + v.element0, element1 + v.element1, element2 + v.element2);
}
Vec3 Vec3::minus(const Vec3 &v) const {
    return Vec3(element0 - v.element0, element1 - v.element1, element2 - v.element2);
}
Vec3 Vec3::multiply(float f) const {
    return Vec3(element0 * f, element1 * f, element2 * f);
}
Vec3 Vec3::divide(float f) const {
    return Vec3(element0 / f, element1 / f, element2 / f);
}


FreeFaller::FreeFaller(float mass, const Vec3 &position, const Vec3 &velocity) :
                m(mass),
                x(position.value(0), position.value(1), position.value(2)),
                v(velocity.value(0), velocity.value(1), velocity.value(2)),
                g(0, -9.8, 0) {}
void FreeFaller::step(float time_step, const Vec3 &forces) {
    Eigen::Vector3d Eforces(forces.value(0), forces.value(1), forces.value(2));
    Eigen::Vector3d f = Eforces + m * g;
    x = x + time_step * v;
    v = v + time_step / m * f;
    /*
    Vec3 f = forces.plus(g.multiply(m));
    x = x.plus(v.multiply(time_step));
    v = v.plus(f.multiply(time_step/m));
    */
}
Vec3 FreeFaller::position() const {
    return Vec3(x(0), x(1), x(2));
}
Vec3 FreeFaller::velocity() const {
    return Vec3(v(0), v(1), v(2));
}

