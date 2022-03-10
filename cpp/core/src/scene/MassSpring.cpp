#include "scene/scene.h"
#include "scene/MassSpring.h"
#include <cmath>
#include <iostream>

Spring::Spring(int i, int j, float l0, float ks, float kd) : i(i), j(j), l0(l0), ks(ks), kd(kd) {}

Eigen::Vector3d Spring::fi(Eigen::Vector3d xi, Eigen::Vector3d vi, Eigen::Vector3d xj, Eigen::Vector3d vj) {
    /*Eigen::Vector3d Exi(xi.value(0), xi.value(1), xi.value(2)),
                    Evi(vi.value(0), vi.value(1), vi.value(2)),
                    Exj(xj.value(0), xj.value(1), xj.value(2)),
                    Evj(vj.value(0), vj.value(1), vj.value(2));
                    */
    Eigen::Vector3d sf = ks * (xj - xi) / (xj - xi).norm() * ((xj - xi).norm() - l0);
    Eigen::Vector3d df = kd * (vj - vi).dot(xj - xi) * (xj - xi) / pow((xj - xi).norm(), 2);
    Eigen::Vector3d force = sf + df;
    return force;
    //return Vec3(force(0), force(1), force(2));
}

Eigen::Vector3d Spring::fj(Eigen::Vector3d xi, Eigen::Vector3d vi, Eigen::Vector3d xj, Eigen::Vector3d vj) {
    /*Eigen::Vector3d Exi(xi.value(0), xi.value(1), xi.value(2)),
                    Evi(vi.value(0), vi.value(1), vi.value(2)),
                    Exj(xj.value(0), xj.value(1), xj.value(2)),
                    Evj(vj.value(0), vj.value(1), vj.value(2));
                    */
    Eigen::Vector3d sf = - ks * (xj - xi) / (xj - xi).norm() * ((xj - xi).norm() - l0);
    Eigen::Vector3d df = - kd * (vj - vi).dot(xj - xi) * (xj - xi) / pow((xj - xi).norm(), 2);
    Eigen::Vector3d force = sf + df;
    return force;
    //return Vec3(force(0), force(1), force(2));
}




MassSpring::MassSpring(int number_object, int number_spring) : no(number_object), ns(number_spring) {
    printf("The system contains %d objects.\n", number_object);
    for (int i = 0; i < number_object; i++) {
        printf("Please input the mass of the %dth object: ", i);
        float mass;
        std::cin >> mass;
        m.push_back(mass);

        printf("Please input the position of the %dth object: ", i);
        float x1, x2, x3;
        std::cin >> x1 >> x2 >> x3;
        Eigen::Vector3d position(x1, x2, x3);
        x.push_back(position);

        printf("Please input the velocity of the %dth object: ", i);
        float v1, v2, v3;
        std::cin >> v1 >> v2 >> v3;
        Eigen::Vector3d velocity(v1, v2, v3);
        v.push_back(velocity);
    }

    printf("\nThe system contains %d springs.\n", number_spring);
    printf("Please input i,j,l0,ks,kd for each spring:\n");
    for (int sn = 0; sn < number_spring; sn++) {
        int i, j; float l0, ks, kd;
        std::cin >> i >> j >> l0 >> ks >> kd;
        Spring spring(i, j, l0, ks, kd);
        S.push_back(spring);
    }
}

float MassSpring::mass(int index) {
    return m[index];
}

Vec3 MassSpring::position(int index) {
    Eigen::Vector3d p = x[index];
    return Vec3(p(0), p(1), p(2));
}

Vec3 MassSpring::velocity(int index) {
    Eigen::Vector3d p = v[index];
    return Vec3(p(0), p(1), p(2));
}

std::vector<Eigen::Vector3d> MassSpring::force() {
    std::vector<Eigen::Vector3d> f;
    for (int i = 0; i < no; i++) {
        Eigen::Vector3d force(0, -9.8, 0);
        f.push_back(force);
    }
    for (int s = 0; s < ns; s++) {
        int i = S[s].i, j = S[s].j;
        f[i] += S[s].fi(x[i], v[i], x[j], v[j]);
        f[j] += S[s].fj(x[i], v[i], x[j], v[j]);
    }
    return f;
}

std::vector<Eigen::Vector3d> MassSpring::force(std::vector<Eigen::Vector3d> x_prime,
                                               std::vector<Eigen::Vector3d> v_prime) {
    std::vector<Eigen::Vector3d> f;
    for (int i = 0; i < no; i++) {
        Eigen::Vector3d force(0, -9.8, 0);
        f.push_back(force);
    }
    for (int s = 0; s < ns; s++) {
        int i = S[s].i, j = S[s].j;
        f[i] += S[s].fi(x_prime[i], v_prime[i], x_prime[j], v_prime[j]);
        f[j] += S[s].fj(x_prime[i], v_prime[i], x_prime[j], v_prime[j]);
    }
    return f;
}


void MassSpring::step_forward(float time_step, int precision) {
    float delta_t = time_step / precision;
    while (precision--) {
        std::vector<Eigen::Vector3d> f = force();
        for (int p = 0; p < no; p++) {
            x[p] = x[p] + delta_t * v[p];
            v[p] = v[p] + delta_t * f[p] / m[p];
        }
    }
}

void MassSpring::step_rk(float time_step, int precision) {
    float delta_t = time_step / precision;
    while(precision--) {
        std::vector<Eigen::Vector3d> a1, a2, b1, b2;

        std::vector<Eigen::Vector3d> f = force();
        for (int p = 0; p < no; p++) {
            a1.push_back(v[p]);
            a2.push_back(f[p] / m[p]);
        }

        std::vector<Eigen::Vector3d> x_prime, v_prime;
        for (int p = 0; p < no; p++) {
            x_prime.push_back(x[p] + delta_t/2 * a1[p]);
            v_prime.push_back(v[p] + delta_t/2 * a2[p]);
        }
        std::vector<Eigen::Vector3d> f_prime = force(x_prime, v_prime);
        for (int p = 0; p < no; p++) {
            b1.push_back(v[p] + delta_t/2 * a2[p]);
            b2.push_back(f_prime[p] / m[p]);
        }

        for (int p = 0; p < no; p++) {
            x[p] = x[p] + delta_t * b1[p];
            v[p] = v[p] + delta_t * b2[p];
        }
    }
}

void MassSpring::step_implicit(float time_step, int precision) {
    float delta_t = time_step / precision;
    while(precision--) {
        Eigen::VectorXd Xt(3 * no), Vt(3 * no), Ft(3 * no);
        std::vector<Eigen::Vector3d> f = force();
        for (int p = 0; p < no; p++) {
            Xt.segment(3*p, 3) = x[p];
            Vt.segment(3*p, 3) = v[p];
            Ft.segment(3*p, 3) = f[p];
        }

        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3*no, 3*no), K = Eigen::MatrixXd::Zero(3*no, 3*no);
        for (int p = 0; p < no; p++) {
            M(3*p, 3*p) = m[p];
            M(3*p+1, 3*p+1) = m[p];
            M(3*p+2, 3*p+2) = m[p];
        }
        for (int s = 0; s < ns; s++) {
            int i = S[s].i, j = S[s].j;
            float l = (x[i] - x[j]).norm();
            float l0 = S[s].l0, ks = S[s].ks;
            Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3, 3);
            Eigen::Matrix3d Kii = ks * (-I + l0/l * (I - (x[j] - x[i]) * (x[j] - x[i]).adjoint() / (l*l) ));
            K.block<3,3>(3*i, 3*i) += Kii;
            K.block<3,3>(3*i, 3*j) += -Kii;
            K.block<3,3>(3*j, 3*i) += -Kii;
            K.block<3,3>(3*j, 3*j) += Kii;
        }

        Eigen::MatrixXd A = M - delta_t * delta_t * K;
        Eigen::VectorXd B = M * Vt + delta_t * Ft;


        Eigen::VectorXd V_prime = A.colPivHouseholderQr().solve(B);
        Eigen::VectorXd X_prime = Xt + delta_t * V_prime;

        for (int p = 0; p < no; p++) {
            v[p] = Eigen::Vector3d(V_prime(3*p), V_prime(3*p+1), V_prime(3*p+2));
            x[p] = Eigen::Vector3d(X_prime(3*p), X_prime(3*p+1), X_prime(3*p+2));
        }
    }
}