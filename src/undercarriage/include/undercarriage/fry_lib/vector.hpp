#pragma once

namespace FRY{
    struct Vec2d{
        float x;
        float y;
        //ベクトル成分
        Vec2d(float x,float y):x(x),y(y){}
        friend auto operator*(const Vec2d a,const Vec2d b) -> float{
            return a.x*b.x+b.y*a.y;
        }//内積
        static Vec2d make(float x,float y){
            return Vec2d{.x = x,.y = y};
        }//ｘ,ｙを要素にもつベクトルを返す
    };
    
}