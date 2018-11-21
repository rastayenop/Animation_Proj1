#version 410
#define M_PI 3.14159265358979323846

uniform mat4 mat_inverse;
uniform mat4 persp_inverse;
uniform sampler2D envMap;
uniform vec3 center;
uniform float radius;

uniform bool transparent;
uniform float shininess;
uniform float eta;

in vec4 position;

out vec4 fragColor;


int nb_reflection_interne = 5;

float getDelta(vec4 u, vec4 CP, float r) {
  return pow(dot(u,CP), 2) - dot(CP,CP) + pow(r, 2);
}

vec4 getNormale(in vec3 interPoint, in vec3 center) {
  vec4 normale;
  normale.xyz = normalize(interPoint - center);
  normale.w = 0;
  return normale;
}

void getRefrRefl(in vec4 normale, in vec4 u, in float eta, out vec4 reflectedRay, out vec4 refractedRay) {
  reflectedRay = reflect(u, normale);
  refractedRay = refract(u, normale, 1/eta);
}

float coeffFresnel(in vec4 x, in vec4 y) {
  float eta2 = pow(eta,2);
  x.xyz = normalize(x.xyz);
  y.xyz = normalize(y.xyz);
  float cosTheta = dot(x.xyz, y.xyz);
  float c_i2 = eta2 + pow(cosTheta, 2) - 1;
  float f_s;
  float f_p;
  if (c_i2 >= 0){
    float c_i = pow(c_i2, 0.5);
    f_s = pow( abs((cosTheta - c_i)/(cosTheta + c_i)), 2);
    f_p = pow( abs((eta2*cosTheta - c_i)/(eta2*cosTheta + c_i)), 2);
    return (f_s + f_p)*0.5;
  }
  else {
    return 1;
  }
}

bool raySphereIntersect(in vec3 start, in vec4 direction, out vec3 newPoint) {
    vec4 CP;
    CP.xyz = center - start;
    CP.w = 0;
    float delta = getDelta(direction, CP, radius);
    float epsilon = 1e-3;
    if (delta > 0){
      float lambda1 = dot(direction, CP) - sqrt(delta);
      if (lambda1 <= epsilon) {
        float lambda2 = dot(direction, CP) + sqrt(delta);
        if (lambda2 > epsilon) {
          newPoint = start + lambda2 * direction.xyz;
          return true;
        }
      }
      else {
        newPoint = start + lambda1 * direction.xyz;
        return true;
      }
    }
    return false;
}

vec4 getColorFromEnvironment(in vec3 direction)
{
    // TODO
    float theta = (atan(direction.y, direction.x) + M_PI)/(2*M_PI);
    float delta = (asin(direction.z) + M_PI*0.5)/(M_PI);
    vec2 coordSphe = vec2(theta, delta);
    return texture2D(envMap, coordSphe);
}




void main(void)
{
    // Step 1: I need pixel coordinates. Division by w?
    vec4 worldPos = position;
    worldPos.z = 1; // near clipping plane
    worldPos = persp_inverse * worldPos;
    worldPos /= worldPos.w;
    worldPos.w = 0;
    worldPos = normalize(worldPos);
    // Step 2: ray direction:
    vec4 u;
    u.xyz = normalize((mat_inverse * worldPos).xyz);
    u.w = 0;
    vec3 eye = (mat_inverse * vec4(0, 0, 0, 1)).xyz;
    vec3 interPoint;
    // TODO
    vec4 resultColor;
    if (raySphereIntersect(eye, u, interPoint)) {
      vec4 normale = getNormale(interPoint, center);
      float coefIntensity = coeffFresnel(-u, normale);
      float coefFresnelLocal;
      vec4 reflectedRay;
      vec4 refractedRay;
      getRefrRefl(normale, u, eta, reflectedRay, refractedRay);
      resultColor = coefIntensity * getColorFromEnvironment(reflectedRay.xyz);
      reflectedRay = refractedRay;
      coefIntensity = 1 - coefIntensity;
      for (int i=0; i<nb_reflection_interne; i++) {
        raySphereIntersect(interPoint, refractedRay, interPoint);
        normale = getNormale(center, interPoint);
        coefFresnelLocal = coeffFresnel(-reflectedRay, normale);
        getRefrRefl(normale, reflectedRay, 1/eta, reflectedRay, refractedRay);
        resultColor += (1 - coefFresnelLocal) * coefIntensity * getColorFromEnvironment(refractedRay.xyz);
        coefIntensity *= coefFresnelLocal;
      }
    }
    else {
      resultColor = getColorFromEnvironment(u.xyz);
    }

    fragColor = resultColor;
}
