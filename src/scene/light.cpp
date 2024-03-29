#include <cmath>

#include "light.h"

using namespace std;

double DirectionalLight::distanceAttenuation(const Vec3d& P) const
{
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}


Vec3d DirectionalLight::shadowAttenuation(const ray& r, const Vec3d& p) const
{
    ray shadowRay(p, getDirection(p), ray::SHADOW);
    
    isect i;
    if(getScene()->intersect(shadowRay, i)) {
        // Opaque objects block light completely
        if(!i.getMaterial().Trans()) {
            return Vec3d(0, 0, 0);
        }
        // Transparent objects attenuate the light
        return prod(i.material->ka(i), color);
    }
    return Vec3d(1,1,1);
}

Vec3d DirectionalLight::getColor() const
{
  return color;
}

Vec3d DirectionalLight::getDirection(const Vec3d& P) const
{
  // for directional light, direction doesn't depend on P
  return -orientation;
}

double PointLight::distanceAttenuation(const Vec3d& P) const
{
    double dist = sqrt( pow(P[0] - position[0], 2) + 
                        pow(P[1] - position[1], 2) + 
                        pow(P[2] - position[2], 2) );

    return min(1.0, 1.0 / (constantTerm + linearTerm * dist + quadraticTerm * dist * dist));
}

Vec3d PointLight::getColor() const
{
  return color;
}

Vec3d PointLight::getDirection(const Vec3d& P) const
{
  Vec3d ret = position - P;
  ret.normalize();
  return ret;
}


Vec3d PointLight::shadowAttenuation(const ray& r, const Vec3d& p) const
{
    ray shadowRay(p, getDirection(p), ray::SHADOW);

    isect i;
    if(getScene()->intersect(shadowRay, i) && (p - position).length() > (p - shadowRay.at(i.t)).length()) {
        // Opaque objects block light completely
        if(!i.getMaterial().Trans()) {
            return Vec3d(0, 0, 0);
        }
        // Transparent objects attenuate the light
        return prod(i.material->ka(i), color);
    }
    return Vec3d(1,1,1);
}
