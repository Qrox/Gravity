#ifndef __GRAVITY_SPACE_H__
#define __GRAVITY_SPACE_H__

#include <algorithm>
#include <vector>
#include <map>
#include <iostream>
#include "vec.h"
#include "advancedmath.h"
#include "types.h"

template <typename T, u32 D>
class particle {
private:
    vec<T, D> p, v, dv;
    T r, m;

public:
    particle(vec<T, D> const & p, vec<T, D> const & v, T r, T m);

    bool intersects(particle<T, D> const & o) const;

    T collision_time_with(particle<T, D> const & o) const;
    void forward(T time);

    void gravity(particle<T, D> & o, T time, T G);
    void collide(particle<T, D> & o);
    void apply();

    vec<T, D> const & position() const;
    vec<T, D> const & velocity() const;
    T radius() const;
    T mass() const;
};

template <typename T, u32 D>
class space {
private:
    std::vector<particle<T, D>> p;

public:
    void forward(T time, T max_displacement_per_step, T G);

    bool add_particle(particle<T, D> const & par);
    bool add_particle(vec<T, D> const & p, vec<T, D> const & v, T r, T m);

    std::vector<particle<T, D>> const & particles() const;
};

template <typename T, u32 D>
particle<T, D>::particle(vec<T, D> const & p, vec<T, D> const & v, T r, T m) : p(p), v(v), dv(0.), r(r), m(m) {
}

template <typename T, u32 D>
bool particle<T, D>::intersects(particle<T, D> const & o) const {
    T mindist = r + o.r;
    return (o.p - p).sqr() < mindist * mindist;
}

template <typename T, u32 D>
T particle<T, D>::collision_time_with(particle<T, D> const & o) const {
   vec<T, D> dp = p - o.p, dv = v - o.v;
   T dpdotdv = dp.dot(dv);
   if (dpdotdv >= 0) return NAN;
   T d = r + o.r;
   T a = dv.sqr(), b = dpdotdv * 2, c = dp.sqr() - d * d;
   T res[2];
   solve_quadric_equation(a, b, c, res);
   // note: any comparison with NAN will return false
   if (res[0] > 0) { // if not NAN and positive
       if (res[1] > 0) { // if not NAN and positive
           return std::min(res[0], res[1]);
       } else { // if NAN or nonpositive
           return res[0];
       }
   } else if (res[1] > 0) { // if not NAN and positive
       return res[1];
   } else { // if NAN or nonpositive
       return NAN;
   }
}

template <typename T, u32 D>
void particle<T, D>::forward(T time) {
    p = p + v * time;
}

template <typename T, u32 D>
void particle<T, D>::gravity(particle<T, D> & o, T time, T G) {
    vec<T, D> dp = o.p - p;
    T k = dp.mod();
    if (k == 0) return;     // avoid division by zero
    k = G * time / (k * k * k);
    dv = dv + dp * (o.m * k);
    o.dv = o.dv + dp * (-m * k);
}

template <typename T, u32 D>
void particle<T, D>::collide(particle<T, D> & o) {
    vec<T, D> dp = (o.p - p).normalize();
    T vad1 = v.dot(dp), vbd1 = o.v.dot(dp);
    T sm = o.m + m, dm = o.m - m;
    if (sm == 0) return;    // avoid division by zero. perhaps treat as collision of same mass instead of return?
    T vad2 = (o.m * vbd1 * 2 - dm * vad1) / sm, vbd2 = (m * vad1 * 2 + dm * vbd1) / sm;
    dv = dv + dp * (vad2 - vad1);
    o.dv = o.dv + dp * (vbd2 - vbd1);
}

template <typename T, u32 D>
void particle<T, D>::apply() {
    v = v + dv;
    dv = vec<T, D>();
}

template <typename T, u32 D>
vec<T, D> const & particle<T, D>::position() const {
    return p;
}

template <typename T, u32 D>
vec<T, D> const & particle<T, D>::velocity() const {
    return v;
}

template <typename T, u32 D>
T particle<T, D>::radius() const {
    return r;
}

template <typename T, u32 D>
T particle<T, D>::mass() const {
    return m;
}

template <typename T, u32 D>
void space<T, D>::forward(T time, T mdps, T G) {
#if defined(DEBUG) && false
    particle<T, D> * ps = &p[0];    // help the silly debugger know where the elements are
#endif
    u32 len = p.size();
    T min_time = time * 1e-3;       // error margin. some magic number?
    T remaining_time = time;
    std::map<u32, std::vector<u32>> collisions;
    while (remaining_time > 0) {
        T forward_time = remaining_time;
        for (u32 i = 0; i < len; ++i) {
            T step_time = mdps / p[i].velocity().mod();                         // if velocity = 0, then step_time = inf, will not affect comparison
            if (step_time < forward_time
                    && remaining_time - step_time < remaining_time) {           // if this subtraction is beyond the precision of T, then ignore it, to avoid a hang
                forward_time = step_time;
            }
        }
//        std::cout << "determining collisions" << std::endl;
        for (u32 i = 0; i < len; ++i) {
            for (u32 j = i + 1; j < len; ++j) {
                T collision_time = p[i].collision_time_with(p[j]);
                if (collision_time < forward_time                               // comparison with NAN will return false
                        && remaining_time - collision_time < remaining_time) {  // if this subtraction is beyond the precision of T, then ignore it, to avoid a hang
                    if (forward_time > collision_time + min_time) collisions.clear();   // error margin
//                    std::cout << "clear" << std::endl;
                    collisions[i].push_back(j);
                    forward_time = collision_time;
//                    std::cout << i << " collides " << j << std::endl;
                } else if (collision_time >= forward_time && collision_time <= forward_time + min_time) {   // error margin
                    collisions[i].push_back(j);
//                    std::cout << i << " collides " << j << std::endl;
                }
            }
        }
//        std::cout << "determined" << std::endl;
//        std::cout << "applying collisions" << std::endl;
        for (u32 i = 0; i < len; ++i) {
            p[i].forward(forward_time);
        }
        for (auto i = collisions.begin(); i != collisions.end(); ++i) {
            u32 _i = i->first;
            std::vector<u32> & col = i->second;
            for (auto j = col.begin(); j < col.end(); ++j) {
                p[_i].collide(p[*j]);
//                std::cout << _i << " collides " << *j << std::endl;
            }
        }
        collisions.clear();
//        std::cout << "applying gravity" << std::endl;
        for (u32 i = 0; i < len; ++i) {
            for (u32 j = i + 1; j < len; ++j) {
                p[i].gravity(p[j], forward_time, G);
            }
        }
        for (u32 i = 0; i < len; ++i) {
            p[i].apply();   // apply changes made by collide() & gravity()
        }
//        std::cout << "applied\n" << std::endl;
        remaining_time -= forward_time;
    }
}

template <typename T, u32 D>
bool space<T, D>::add_particle(particle<T, D> const & par) {
    for (auto i = p.begin(); i < p.end(); ++i) {
        if (i->intersects(par)) return false;
    }
    p.push_back(par);
    return true;
}

template <typename T, u32 D>
bool space<T, D>::add_particle(vec<T, D> const & p, vec<T, D> const & v, T r, T m) {
    return add_particle(particle<T, D>(p, v, r, m));
}

template <typename T, u32 D>
std::vector<particle<T, D>> const & space<T, D>::particles() const {
    return p;
}

#endif
