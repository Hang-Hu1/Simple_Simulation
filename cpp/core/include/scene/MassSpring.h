#ifndef MassSpring_H
#define MassSpring_H

#include "Eigen/SparseLU"
#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include "Eigen/Dense"
#include <vector>


class Spring {
public:
    Spring(int, int, float, float, float);
    Eigen::Vector3d fi(Eigen::Vector3d xi, Eigen::Vector3d vi, Eigen::Vector3d xj, Eigen::Vector3d vj);
    Eigen::Vector3d fj(Eigen::Vector3d xi, Eigen::Vector3d vi, Eigen::Vector3d xj, Eigen::Vector3d vj);

    int i;
    int j;
    float l0;
    float ks;
    float kd;
};

class MassSpring {
public:
    MassSpring(int number_object, int number_spring);
    std::vector<Eigen::Vector3d> force();
    std::vector<Eigen::Vector3d> force(std::vector<Eigen::Vector3d> x_prime, std::vector<Eigen::Vector3d> v_prime);

    void step_forward(float time_step, int precision);
    void step_rk(float time_step, int precision);
    void step_implicit(float time_step, int precision);

    float mass(int index);
    Vec3 position(int index);
    Vec3 velocity(int index);

private:
    int no, ns;
    std::vector<float> m;
    std::vector<Eigen::Vector3d> x;
    std::vector<Eigen::Vector3d> v;
    std::vector<Spring> S;
};


#endif