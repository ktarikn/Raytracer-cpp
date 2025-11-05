
#include "parser.h"
#include <cmath>

#define Vec3f parser::Vec3f

// Vector operations kept here for tidyness

struct Ray { 
	Vec3f o, d; 
	Ray(){} Ray(const Vec3f& o_, const Vec3f& d_):o(o_),d(d_){}
};

// honestly I did not *feel* the 'improvement'
float Q_rsqrt( float number ) //taken from scrapped codes of Quake, thanks John Carmack
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the *no cursing*
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}


float dot(Vec3f a, Vec3f b){
    float result = 0;
    result+= a.x*b.x;
    result+= a.y*b.y;
    result+= a.z*b.z;
    return result;
}

Vec3f cross(Vec3f a, Vec3f b){
    Vec3f result;
    result.x = a.y*b.z - a.z*b.y;
    result.y = a.z*b.x - a.x*b.z;
    result.z = a.x*b.y - a.y*b.x;
    return result;
}
float vLength(Vec3f vec){
    
    return 1/Q_rsqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
    
}
Vec3f normalize(Vec3f vec){
    Vec3f result;
    float sqrsum = vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
    float invLen = Q_rsqrt(sqrsum);
    //float len = vLength(vec);
    result.x = vec.x*invLen;
    result.y = vec.y*invLen;
    result.z = vec.z*invLen;
    return result;
}
Vec3f constMult(Vec3f a, float c){
    Vec3f result;
    result.x = a.x*c;
    result.y = a.y*c;
    result.z = a.z*c;
    return result;
}
Vec3f add(Vec3f a, Vec3f b){
    Vec3f result;
    result.x = a.x+b.x;
    result.y = a.y+b.y;
    result.z = a.z+b.z;
    return result;
}
Vec3f subtract(Vec3f a, Vec3f b){
    Vec3f result;
    result.x = a.x-b.x;
    result.y = a.y-b.y;
    result.z = a.z-b.z;
    return result;
}
bool isZero(Vec3f vec){
    return vec.x==0 && vec.y == 0 & vec.z == 0;
}

//ohhhh this existed :(
Vec3f reflect(const Vec3f& I, const Vec3f& N) {
    return subtract(I ,constMult(N,2.0f*dot(I,N)));
}