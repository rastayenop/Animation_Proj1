#version 410

#define M_PI 3.14159265358979323846

uniform float lightIntensity;
uniform bool blinnPhong;
uniform float shininess;
uniform float eta;
uniform sampler2D shadowMap;

in vec4 eyeVector;
in vec4 lightVector;
in vec4 vertColor;
in vec4 vertNormal;
in vec4 lightSpace;

vec4 C_a = vec4(0.0,0.0,0.0,1.0);
vec4 C_d = vec4(0.0,0.0,0.0,1.0);
vec4 C_s = vec4(0.0,0.0,0.0,1.0);
float k_a = 0.1;
float k_d = 0.5;
float alpha = 1/shininess;

vec4 eyeVector1 = normalize(eyeVector);
vec4 lightVector1 = normalize(lightVector);
vec4 vertNormal1 = normalize(vertNormal);


out vec4 fragColor;


float g1 (float cos_theta)
{
    float g1_theta = 2/(1 + pow(1 + pow(alpha,2)*((1 - pow(cos_theta,2))/pow(cos_theta,2)), 0.5));
    return g1_theta;
}

int ki (float cos_theta)
{
    if (cos_theta > 0) {return 1;}
    else {return 0;}
}


float d_theta(float cos_theta)
{
    return ki(cos_theta) * pow(alpha, 2) / (M_PI * pow(cos_theta, 4) * pow(pow(alpha,2) + (1 - pow(cos_theta,2))/pow(cos_theta,2), 2));
}



float fTheta (vec4 H, vec4 lightVector) {
  float cos_theta = dot(H, lightVector);
  float c_i_theta = pow(pow(eta, 2) - (1 - pow(cos_theta, 2)), 0.5);
  float f_s = pow((cos_theta - c_i_theta) / (cos_theta + c_i_theta), 2);
  float f_p = pow((pow(eta,2)*cos_theta - c_i_theta) / (pow(eta,2)*cos_theta + c_i_theta), 2);
  return (f_s + f_p)*0.5;
}

void main( void )
{
     // Ambient light
     C_a = k_a * vertColor * lightIntensity;
     // Diffuse light
     C_d = k_d * vertColor * max(dot(vertNormal1, lightVector1),0) * lightIntensity;
     // Specular light
     vec4 H = normalize(eyeVector + lightVector);
     if (blinnPhong) {
       C_s = fTheta(H, lightVector1) * vertColor * pow(max(dot(vertNormal1, H), 0), shininess) * lightIntensity;
     }
     else {
       float cos_theta_i = dot(vertNormal1, lightVector1);
       float cos_theta_o = dot(vertNormal1, eyeVector1);
       float cos_theta_h = dot(vertNormal1, H);
       C_s = fTheta(eyeVector1, lightVector1) * d_theta(cos_theta_h) * g1(cos_theta_i) * g1(cos_theta_o) / (4 * cos_theta_i * cos_theta_o) * vertColor * lightIntensity;
     }
     // This is the place where there's work to be done
     fragColor = C_a + C_d + C_s;
}
